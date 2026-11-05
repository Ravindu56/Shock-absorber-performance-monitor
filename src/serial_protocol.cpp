#include "serial_protocol.h"

#include <ctype.h>
#include <mbedtls/base64.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace proto {

namespace {

// Only the reporting task writes to Serial, so one static buffer is safe.
char g_out[1024];

void emitf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(g_out, sizeof(g_out) - 1, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > int(sizeof(g_out)) - 2) n = sizeof(g_out) - 2;
  g_out[n++] = '\n';
  Serial.write(reinterpret_cast<const uint8_t*>(g_out), n);
}

const char AXIS_NAMES[3] = {'X', 'Y', 'Z'};

void upcase(char* s) {
  for (; *s; ++s) *s = char(toupper(uint8_t(*s)));
}

bool parseFloat(const char* s, float lo, float hi, float& out) {
  char* end = nullptr;
  const float v = strtof(s, &end);
  if (end == s || *end != '\0' || !(v >= lo && v <= hi)) return false;
  out = v;
  return true;
}

bool parseUint(const char* s, uint32_t lo, uint32_t hi, uint32_t& out) {
  char* end = nullptr;
  const unsigned long v = strtoul(s, &end, 10);
  if (end == s || *end != '\0' || v < lo || v > hi) return false;
  out = uint32_t(v);
  return true;
}

bool setKey(const char* key, const char* val, RuntimeConfig& c, char* err, size_t errlen) {
  uint32_t u;
  float f;
  if (!strcmp(key, "PROFILE")) {
    for (uint8_t i = 0; i < PROFILE_COUNT; ++i) {
      if (!strcasecmp(val, SENSOR_PROFILES[i].name)) {
        c.profile = i;
        return true;
      }
    }
    if (parseUint(val, 0, PROFILE_COUNT - 1, u)) {
      c.profile = uint8_t(u);
      return true;
    }
    snprintf(err, errlen, "unknown profile %s", val);
    return false;
  }
  if (!strcmp(key, "RANGE")) {
    for (uint8_t i = 0; i < 4; ++i) {
      if (parseUint(val, 0, 99, u) && u == RANGE_G[i]) {
        c.range = i;
        return true;
      }
    }
    snprintf(err, errlen, "range must be 2, 4, 8 or 16");
    return false;
  }
  if (!strcmp(key, "AXIS")) {
    const char ch = char(toupper(uint8_t(val[0])));
    if (val[1] == '\0' && ch >= 'X' && ch <= 'Z') {
      c.primary_axis = uint8_t(ch - 'X');
      return true;
    }
    if (parseUint(val, 0, 2, u)) {
      c.primary_axis = uint8_t(u);
      return true;
    }
    snprintf(err, errlen, "axis must be X, Y, Z or 0..2");
    return false;
  }
  if (!strcmp(key, "FFT_N")) {
    if (parseUint(val, 256, cfg::FFT_MAX_N, u) && (u & (u - 1)) == 0) {
      c.fft_n = u;
      return true;
    }
    snprintf(err, errlen, "fft_n must be a power of two in 256..%u", unsigned(cfg::FFT_MAX_N));
    return false;
  }

  struct FloatKey { const char* key; float* dst; float lo, hi; };
  const FloatKey fkeys[] = {
      {"TAU", &c.baseline_tau_s, 0.05f, 30.f},
      {"THRESHOLD", &c.impact_threshold, 0.05f, 1000.f},
      {"HYSTERESIS", &c.impact_hysteresis, 0.f, 1000.f},
      {"FMIN", &c.fft_fmin_hz, 0.f, 500.f},
      {"OFS_X", &c.offset[0], -20.f, 20.f},  // m/s^2 (a 16 g part is spec'd to +-0.6 g ZG offset)
      {"OFS_Y", &c.offset[1], -20.f, 20.f},
      {"OFS_Z", &c.offset[2], -20.f, 20.f},
      {"SCL_X", &c.scale[0], 0.8f, 1.2f},    // sensitivity tolerance is a few %
      {"SCL_Y", &c.scale[1], 0.8f, 1.2f},
      {"SCL_Z", &c.scale[2], 0.8f, 1.2f},
      {"TARE_X", &c.tare[0], -200.f, 200.f},  // zero offsets (0 = show gravity again)
      {"TARE_Y", &c.tare[1], -200.f, 200.f},
      {"TARE_Z", &c.tare[2], -200.f, 200.f},
  };
  for (const FloatKey& k : fkeys) {
    if (strcmp(key, k.key)) continue;
    if (parseFloat(val, k.lo, k.hi, f)) {
      *k.dst = f;
      return true;
    }
    snprintf(err, errlen, "%s must be %g..%g", key, double(k.lo), double(k.hi));
    return false;
  }

  struct UintKey { const char* key; uint32_t* dst; uint32_t lo, hi; };
  uint32_t stream = c.stream_raw ? 1 : 0;
  const UintKey ukeys[] = {
      {"RMS_MS", &c.rms_window_ms, 10, 10000},
      {"PERSIST", &c.impact_persist_samples, 1, 100},
      {"WINDOW_MS", &c.event_window_ms, 1, 10000},
      {"RELEASE_MS", &c.release_hold_ms, 0, 1000},
      {"REFRACTORY_MS", &c.refractory_ms, 0, 60000},
      {"REPORT_HZ", &c.report_hz, 1, 100},
      {"FFT_MS", &c.fft_interval_ms, 200, 60000},
      {"STREAM", &stream, 0, 1},
  };
  for (const UintKey& k : ukeys) {
    if (strcmp(key, k.key)) continue;
    if (parseUint(val, k.lo, k.hi, u)) {
      *k.dst = u;
      c.stream_raw = stream != 0;
      return true;
    }
    snprintf(err, errlen, "%s must be an integer %u..%u", key, unsigned(k.lo), unsigned(k.hi));
    return false;
  }
  snprintf(err, errlen, "unknown key %s", key);
  return false;
}

}  // namespace

void sendHello() {
  emitf("HELLO,%d,%s,0x%02X,0x%02X,%s", PROTOCOL_VERSION, FIRMWARE_VERSION, acq::sensorAddress(),
        acq::sensorWhoAmI(), acq::initError());
}

void sendHeaders() {
  emitf("H,S,idx,t_ms,ax,ay,az,amag,dx,dy,dz,dmag,rms_win,p2p_win,peak_int,test_max_dyn,test_rms,"
        "test_p2p,impacts,impact_active,fs_meas,flags,fifo_ovf,i2c_err,stream_drop,gaps,dup_ratio");
  emitf("H,TS,idx,samples,max_x,min_x,max_y,min_y,max_z,min_z,max_dyn,max_dprim,min_dprim,rms,p2p,"
        "impacts,clipped");
  emitf("H,E,id,start_idx,peak_idx,end_idx,t_start_ms,peak_dyn,peak_dx,peak_dy,peak_dz,duration_ms,"
        "interval_ms,truncated");
  emitf("H,F,idx,n,fs,df,fmin,fmax,dom_hz,dom_amp,noise_floor,valid");
  emitf("H,R,gen,first_idx,n,b64_int16le_xyz");
  emitf("H,CAL,samples,mean_x,mean_y,mean_z,std_x,std_y,std_z,mag_mean,roll_deg,pitch_deg,clipped,"
        "fs_meas");
}

void sendConfig(const RuntimeConfig& c, uint32_t gen) {
  const SensorProfile& p = SENSOR_PROFILES[c.profile];
  emitf("CFG,gen=%u,profile=%s,odr_hz=%.1f,bw_hz=%.1f,valid_max_hz=%.1f,alias_risk=%d,range_g=%u,"
        "lsb_per_g=%.1f,baseline_tau_s=%.3f,rms_window_ms=%u,primary_axis=%c,threshold=%.3f,"
        "hysteresis=%.3f,persist=%u,event_window_ms=%u,release_hold_ms=%u,refractory_ms=%u,"
        "report_hz=%u,stream=%d,fft_n=%u,fft_interval_ms=%u,fft_fmin_hz=%.2f,ofs_x=%.5f,"
        "ofs_y=%.5f,ofs_z=%.5f,scl_x=%.5f,scl_y=%.5f,scl_z=%.5f,tare_x=%.5f,tare_y=%.5f,"
        "tare_z=%.5f,units=m/s2",
        unsigned(gen & 0xFF), p.name, double(p.odr_hz), double(p.bw_hz), double(p.valid_max_hz),
        p.alias_risk ? 1 : 0, unsigned(RANGE_G[c.range & 3]), double(RANGE_LSB_PER_G[c.range & 3]),
        double(c.baseline_tau_s), unsigned(c.rms_window_ms), AXIS_NAMES[c.primary_axis % 3],
        double(c.impact_threshold), double(c.impact_hysteresis), unsigned(c.impact_persist_samples),
        unsigned(c.event_window_ms), unsigned(c.release_hold_ms), unsigned(c.refractory_ms),
        unsigned(c.report_hz), c.stream_raw ? 1 : 0, unsigned(c.fft_n), unsigned(c.fft_interval_ms),
        double(c.fft_fmin_hz), double(c.offset[0]), double(c.offset[1]), double(c.offset[2]),
        double(c.scale[0]), double(c.scale[1]), double(c.scale[2]), double(c.tare[0]),
        double(c.tare[1]), double(c.tare[2]));
}

void sendStatus(const AcqSnapshot& s) {
  emitf("S,%u,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%d,"
        "%.2f,%u,%u,%u,%u,%u,%.3f",
        unsigned(s.idx), double(s.t_us) / 1000.0, double(s.a[0]), double(s.a[1]), double(s.a[2]),
        double(s.amag), double(s.d[0]), double(s.d[1]), double(s.d[2]), double(s.dmag),
        double(s.win.rms), double(s.win.p2p), double(s.peak_interval), double(s.test.max_dyn),
        double(s.test.rms()), double(s.test.p2p()), unsigned(s.impacts), s.impact_active ? 1 : 0,
        double(s.fs_meas), unsigned(s.flags), unsigned(s.fifo_overflows), unsigned(s.i2c_errors),
        unsigned(s.stream_drops), unsigned(s.gaps), double(s.dup_ratio));
}

void sendTestStats(const AcqSnapshot& s) {
  const dsp::TestStats& t = s.test;
  emitf("TS,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u", unsigned(s.idx),
        unsigned(t.samples), double(t.max_raw[0]), double(t.min_raw[0]), double(t.max_raw[1]),
        double(t.min_raw[1]), double(t.max_raw[2]), double(t.min_raw[2]), double(t.max_dyn),
        double(t.max_dprimary), double(t.min_dprimary), double(t.rms()), double(t.p2p()),
        unsigned(s.impacts), unsigned(t.clipped));
}

void sendEvent(const ImpactEvent& e) {
  emitf("E,%u,%u,%u,%u,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%d", unsigned(e.id),
        unsigned(e.start_idx), unsigned(e.peak_idx), unsigned(e.end_idx),
        double(e.t_start_us) / 1000.0, double(e.peak_dyn), double(e.peak_vec[0]),
        double(e.peak_vec[1]), double(e.peak_vec[2]), double(e.duration_ms),
        double(e.interval_ms), e.truncated ? 1 : 0);
}

void sendSpectrum(uint32_t idx, size_t n, float fs, float fmin, float fmax,
                  const dsp::SpectrumPeak& p, bool valid) {
  emitf("F,%u,%u,%.2f,%.4f,%.2f,%.2f,%.3f,%.5f,%.5f,%d", unsigned(idx), unsigned(n), double(fs),
        double(p.df_hz), double(fmin), double(fmax), double(p.freq_hz), double(p.amp),
        double(p.noise_floor), valid ? 1 : 0);
}

void sendRaw(const RawSample* s, size_t n) {
  if (n == 0) return;
  // Pack int16 little-endian X, Y, Z per sample, then base64 (keeps the link line-based).
  static uint8_t bin[cfg::STREAM_BLOCK_SAMPLES * 6];
  static unsigned char b64[((cfg::STREAM_BLOCK_SAMPLES * 6 + 2) / 3) * 4 + 1];
  if (n > cfg::STREAM_BLOCK_SAMPLES) n = cfg::STREAM_BLOCK_SAMPLES;
  for (size_t i = 0; i < n; ++i) {
    const int16_t v[3] = {s[i].x, s[i].y, s[i].z};
    for (int k = 0; k < 3; ++k) {
      bin[6 * i + 2 * k] = uint8_t(uint16_t(v[k]) & 0xFF);
      bin[6 * i + 2 * k + 1] = uint8_t(uint16_t(v[k]) >> 8);
    }
  }
  size_t olen = 0;
  if (mbedtls_base64_encode(b64, sizeof(b64), &olen, bin, n * 6) != 0) return;
  b64[olen] = '\0';
  emitf("R,%u,%u,%u,%s", unsigned(s[0].gen), unsigned(s[0].idx), unsigned(n), b64);
}

void sendCalibration(const CalResult& c) {
  emitf("CAL,%u,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.2f,%.2f,%u,%.2f", unsigned(c.samples),
        double(c.mean[0]), double(c.mean[1]), double(c.mean[2]), double(c.stdev[0]),
        double(c.stdev[1]), double(c.stdev[2]), double(c.mag_mean), double(c.roll_deg),
        double(c.pitch_deg), unsigned(c.clipped), double(c.fs_meas));
}

void sendAck(const char* cmd, bool ok, const char* msg) {
  emitf("ACK,%s,%s,%s", cmd, ok ? "OK" : "ERR", msg ? msg : "");
}

void sendError(const char* code, const char* msg) { emitf("ERR,%s,%s", code, msg ? msg : ""); }

void sendLog(const char* msg) { emitf("# %s", msg); }

bool readLine(char* line, size_t cap) {
  static char buf[128];
  static size_t len = 0;
  static bool overflow = false;
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) break;
    if (ch == '\r') continue;
    if (ch == '\n') {
      const bool ok = !overflow && len > 0;
      buf[len] = '\0';
      if (ok) strlcpy(line, buf, cap);
      if (overflow) sendError("CMD_TOO_LONG", "command line exceeded 127 characters");
      len = 0;
      overflow = false;
      if (ok) return true;
      continue;
    }
    if (len < sizeof(buf) - 1) {
      buf[len++] = char(ch);
    } else {
      overflow = true;
    }
  }
  return false;
}

ParsedCommand parseCommand(char* line, RuntimeConfig& desired) {
  ParsedCommand pc;
  char* save = nullptr;
  char* word = strtok_r(line, " \t,", &save);
  if (!word) return pc;
  upcase(word);
  strlcpy(pc.name, word, sizeof(pc.name));

  if (!strcmp(word, "PING")) pc.cmd = Command::PING;
  else if (!strcmp(word, "HELLO")) pc.cmd = Command::HELLO;
  else if (!strcmp(word, "GETCFG")) pc.cmd = Command::GETCFG;
  else if (!strcmp(word, "START")) pc.cmd = Command::START;
  else if (!strcmp(word, "STOP")) pc.cmd = Command::STOP;
  else if (!strcmp(word, "RESET")) pc.cmd = Command::RESET;
  else if (!strcmp(word, "SAVE")) pc.cmd = Command::SAVE;
  else if (!strcmp(word, "TARE")) pc.cmd = Command::TARE;
  else if (!strcmp(word, "DEFAULTS")) pc.cmd = Command::DEFAULTS;
  else if (!strcmp(word, "CAL")) {
    pc.cmd = Command::CAL;
    const char* arg = strtok_r(nullptr, " \t,", &save);
    if (arg && !parseUint(arg, 200, 30000, pc.cal_ms)) {
      pc.cmd = Command::INVALID;
      snprintf(pc.error, sizeof(pc.error), "CAL duration must be 200..30000 ms");
    }
  } else if (!strcmp(word, "SET")) {
    char* key = strtok_r(nullptr, " \t,=", &save);
    char* val = strtok_r(nullptr, " \t,=", &save);
    if (!key || !val) {
      pc.cmd = Command::INVALID;
      snprintf(pc.error, sizeof(pc.error), "usage: SET <key> <value>");
      return pc;
    }
    upcase(key);
    RuntimeConfig trial = desired;
    if (!setKey(key, val, trial, pc.error, sizeof(pc.error))) {
      pc.cmd = Command::INVALID;
      return pc;
    }
    if (trial.impact_hysteresis >= trial.impact_threshold) {
      pc.cmd = Command::INVALID;
      snprintf(pc.error, sizeof(pc.error), "hysteresis must be below threshold");
      return pc;
    }
    desired = trial;
    pc.cmd = Command::SET;
  } else {
    pc.cmd = Command::INVALID;
    snprintf(pc.error, sizeof(pc.error), "unknown command");
  }
  return pc;
}

}  // namespace proto
