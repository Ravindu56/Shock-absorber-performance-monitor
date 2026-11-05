#include "acquisition.h"

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <math.h>

#include <atomic>

#include "mpu6500_driver.h"

namespace {

// ----------------------------- shared state --------------------------------
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
AcqSnapshot g_snap;             // guarded by g_mux
RuntimeConfig g_cfg;            // guarded by g_mux
uint32_t g_gen = 0;             // guarded by g_mux
const char* g_init_error = "NOT_STARTED";  // guarded by g_mux
bool g_init_changed = false;    // guarded by g_mux
uint8_t g_addr = 0, g_who = 0;  // guarded by g_mux

// Raw stream SPSC ring: producer = acquisition task, consumer = reporting task.
RawSample g_stream[cfg::STREAM_RING_SAMPLES];
std::atomic<uint32_t> g_stream_head{0};  // next write position (producer)
std::atomic<uint32_t> g_stream_tail{0};  // next read position (consumer)

// FFT ring of primary-axis dynamic acceleration, guarded by g_mux.
float g_fft_ring[cfg::FFT_RING];
uint32_t g_fft_written = 0;    // total samples written since last clear
uint32_t g_fft_contiguous = 0; // samples written since the last gap

QueueHandle_t g_cmd_q = nullptr;
QueueHandle_t g_event_q = nullptr;
QueueHandle_t g_cal_q = nullptr;
QueueHandle_t g_tare_q = nullptr;

// ------------------------ acquisition-task-private -------------------------
Mpu6500 g_mpu;
RuntimeConfig t_cfg;           // task's working copy of the configuration
uint8_t t_gen8 = 0;
bool t_sensor_ok = false;
bool t_running = true;         // acquisition starts automatically after boot
uint32_t t_idx = 0;            // next sample index
int64_t t_last_sample_us = 0;
uint32_t t_consec_err = 0;
uint32_t t_flags_latched = 0;
float t_lsb_per_g = 2048.f;
float t_fs_nom = 1000.f;

dsp::BaselineFilter t_base;
dsp::WindowStats t_win;
dsp::TestStats t_test;
ImpactDetector t_det;

// Sample-rate / duplicate measurement over ~1 s windows
int64_t t_rate_t0 = 0;
uint32_t t_rate_n = 0, t_dup_n = 0;
float t_fs_meas = 0, t_dup_ratio = 0;
int16_t t_prev_raw[3] = {0, 0, 0};

// Calibration accumulator
bool t_cal_active = false;
int64_t t_cal_end_us = 0;
uint32_t t_cal_n = 0, t_cal_clip = 0;
double t_cal_sum[3], t_cal_sumsq[3];

// Zero (tare) accumulator: calibrated monitored-axis acceleration before tare
bool t_tare_active = false;
int64_t t_tare_start_us = 0, t_tare_end_us = 0;
uint32_t t_tare_n = 0;
double t_tare_sum = 0, t_tare_sumsq = 0;

uint8_t t_buf[mpu::FIFO_SIZE_BYTES + 8];

void setInitState(const char* err) {
  portENTER_CRITICAL(&g_mux);
  if (g_init_error != err) g_init_changed = true;
  g_init_error = err;
  g_addr = g_mpu.address();
  g_who = g_mpu.whoAmI();
  portEXIT_CRITICAL(&g_mux);
}

void applyDspConfig() {
  const float fs = t_fs_nom;
  t_base.configure(fs, t_cfg.baseline_tau_s);
  t_win.configure(uint32_t(lroundf(t_cfg.rms_window_ms * fs / 1000.f)));
  ImpactParams p;
  p.threshold = t_cfg.impact_threshold;
  p.hysteresis = t_cfg.impact_hysteresis;
  p.persist_samples = t_cfg.impact_persist_samples;
  p.window_samples = uint32_t(lroundf(t_cfg.event_window_ms * fs / 1000.f));
  p.release_hold_samples = uint32_t(lroundf(t_cfg.release_hold_ms * fs / 1000.f));
  p.refractory_samples = uint32_t(lroundf(t_cfg.refractory_ms * fs / 1000.f));
  t_det.configure(p, fs);
}

void publishConfig() {
  portENTER_CRITICAL(&g_mux);
  g_cfg = t_cfg;
  ++g_gen;
  t_gen8 = uint8_t(g_gen);
  portEXIT_CRITICAL(&g_mux);
}

void clearFftRing() {
  portENTER_CRITICAL(&g_mux);
  g_fft_written = 0;
  g_fft_contiguous = 0;
  portEXIT_CRITICAL(&g_mux);
}

// Mark a discontinuity: advance the sample index by the estimated number of
// samples that were never read so idx / fs keeps tracking elapsed time.
void markGap(int64_t now_us) {
  t_win.reset();
  t_rate_t0 = 0;  // restart rate measurement
  if (t_last_sample_us == 0) return;  // nothing acquired yet: not a gap
  const int64_t missing = llround(double(now_us - t_last_sample_us) * t_fs_nom * 1e-6) - 1;
  if (missing > 0) t_idx += uint32_t(missing);
  t_last_sample_us = now_us;
  t_flags_latched |= ST_GAP;
  portENTER_CRITICAL(&g_mux);
  g_snap.gaps++;
  g_fft_contiguous = 0;
  portEXIT_CRITICAL(&g_mux);
}

bool initSensor() {
  MpuError e = g_mpu.begin(Wire);
  if (e == MpuError::OK) e = g_mpu.configure(t_cfg.profile, t_cfg.range);
  if (e != MpuError::OK) {
    t_sensor_ok = false;
    setInitState(mpuErrorName(e));
    return false;
  }
  t_lsb_per_g = RANGE_LSB_PER_G[t_cfg.range & 3];
  t_fs_nom = SENSOR_PROFILES[t_cfg.profile].odr_hz;
  applyDspConfig();
  t_base.reset();
  t_sensor_ok = true;
  t_consec_err = 0;
  setInitState("OK");
  return true;
}

void onI2cError() {
  t_flags_latched |= ST_I2C_ERR;
  portENTER_CRITICAL(&g_mux);
  g_snap.i2c_errors++;
  portEXIT_CRITICAL(&g_mux);
  if (++t_consec_err >= cfg::I2C_ERRORS_BEFORE_RECOVERY) {
    t_sensor_ok = false;  // the task loop will re-initialise bus + sensor
    portENTER_CRITICAL(&g_mux);
    g_snap.recoveries++;
    portEXIT_CRITICAL(&g_mux);
    setInitState("I2C_RECOVERY");
  }
}

void pushStream(const int16_t r[3], uint32_t idx) {
  if (!t_cfg.stream_raw) return;
  const uint32_t head = g_stream_head.load(std::memory_order_relaxed);
  const uint32_t tail = g_stream_tail.load(std::memory_order_acquire);
  if (head - tail >= cfg::STREAM_RING_SAMPLES) {
    t_flags_latched |= ST_STREAM_DROP;
    portENTER_CRITICAL(&g_mux);
    g_snap.stream_drops++;
    portEXIT_CRITICAL(&g_mux);
    return;
  }
  RawSample& s = g_stream[head % cfg::STREAM_RING_SAMPLES];
  s.x = r[0];
  s.y = r[1];
  s.z = r[2];
  s.gen = t_gen8;
  s.idx = idx;
  g_stream_head.store(head + 1, std::memory_order_release);
}

void processBatch(const uint8_t* buf, size_t frames, int64_t t_read_us) {
  const float scale = cfg::G_TO_MS2 / t_lsb_per_g;  // counts -> m/s^2 (datasheet sensitivity)
  const float period_us = 1e6f / (t_fs_meas > 0 ? t_fs_meas : t_fs_nom);
  float a[3], a_uncal[3], d[3], dmag = 0, amag = 0;
  float batch_peak = 0;
  dsp::WindowResult win;
  bool win_done = false;
  int64_t t_us = t_read_us;

  for (size_t k = 0; k < frames; ++k) {
    const uint8_t* f = buf + k * mpu::FIFO_FRAME_BYTES;
    int16_t r[3];
    bool clip = false, dup = true;
    for (int i = 0; i < 3; ++i) {
      r[i] = int16_t((uint16_t(f[2 * i]) << 8) | f[2 * i + 1]);
      clip |= (r[i] == INT16_MAX || r[i] == INT16_MIN);
      dup &= (r[i] == t_prev_raw[i]);
      t_prev_raw[i] = r[i];
      a_uncal[i] = float(r[i]) * scale;
      a[i] = (a_uncal[i] - t_cfg.offset[i]) * t_cfg.scale[i] - t_cfg.tare[i];
    }
    // The FIFO holds evenly spaced samples (sensor clock); the newest frame was
    // captured just before the read, older ones one period apart.
    t_us = t_read_us - int64_t(float(frames - 1 - k) * period_us);

    t_base.update(a, d);
    dmag = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const float dprim = d[t_cfg.primary_axis % 3];
    // Vibration level, peaks and impacts use the primary (stroke) axis only
    // (Z by default), so lateral X/Y motion cannot trigger events or inflate RMS.
    const float dvib = fabsf(dprim);

    if (t_win.add(dvib, dprim, win)) win_done = true;
    t_test.add(a, dvib, dprim, clip);
    if (dvib > batch_peak) batch_peak = dvib;
    if (clip) t_flags_latched |= ST_CLIP;
    if (dup) ++t_dup_n;

    if (t_tare_active && t_us >= t_tare_start_us) {
      const double v = double(a[t_cfg.primary_axis % 3]) + t_cfg.tare[t_cfg.primary_axis % 3];
      t_tare_sum += v;
      t_tare_sumsq += v * v;
      ++t_tare_n;
    }

    // Impact detection pauses while zeroing: pressing the button shakes the sensor.
    ImpactEvent ev;
    if (!t_tare_active && t_det.update(t_idx, t_us, dvib, d, ev)) {
      if (xQueueSend(g_event_q, &ev, 0) != pdTRUE) {
        portENTER_CRITICAL(&g_mux);
        g_snap.events_dropped++;
        portEXIT_CRITICAL(&g_mux);
      }
    }

    if (t_cal_active) {
      // Uncalibrated values, so the 6-position procedure does not compound an
      // existing calibration. The host applies offset/scale for display.
      for (int i = 0; i < 3; ++i) {
        t_cal_sum[i] += a_uncal[i];
        t_cal_sumsq[i] += double(a_uncal[i]) * a_uncal[i];
      }
      ++t_cal_n;
      if (clip) ++t_cal_clip;
    }

    pushStream(r, t_idx);
    g_fft_ring[t_idx % cfg::FFT_RING] = dprim;  // single writer; readers copy under g_mux
    ++t_idx;
  }
  t_last_sample_us = t_us;

  // Sample-rate measurement over >= 1 s using the ESP32 clock.
  t_rate_n += frames;
  if (t_rate_t0 == 0) {
    t_rate_t0 = t_read_us;
    t_rate_n = 0;
    t_dup_n = 0;
  } else if (t_read_us - t_rate_t0 >= 1000000) {
    const float fs = float(t_rate_n) * 1e6f / float(t_read_us - t_rate_t0);
    t_fs_meas = (t_fs_meas == 0) ? fs : 0.75f * t_fs_meas + 0.25f * fs;
    t_dup_ratio = t_rate_n ? float(t_dup_n) / float(t_rate_n) : 0;
    t_rate_t0 = t_read_us;
    t_rate_n = 0;
    t_dup_n = 0;
  }

  // Calibration completion
  CalResult cal;
  bool cal_done = false;
  if (t_cal_active && t_read_us >= t_cal_end_us && t_cal_n > 1) {
    cal.samples = t_cal_n;
    for (int i = 0; i < 3; ++i) {
      const double m = t_cal_sum[i] / t_cal_n;
      const double var = t_cal_sumsq[i] / t_cal_n - m * m;
      cal.mean[i] = float(m);
      cal.stdev[i] = float(sqrt(var > 0 ? var : 0));
    }
    cal.mag_mean = sqrtf(cal.mean[0] * cal.mean[0] + cal.mean[1] * cal.mean[1] +
                         cal.mean[2] * cal.mean[2]);
    // Static tilt from the gravity vector (aerospace convention, X forward, Z down/up).
    cal.roll_deg = atan2f(cal.mean[1], cal.mean[2]) * 57.29578f;
    cal.pitch_deg = atan2f(-cal.mean[0], sqrtf(cal.mean[1] * cal.mean[1] + cal.mean[2] * cal.mean[2])) * 57.29578f;
    cal.clipped = t_cal_clip;
    cal.fs_meas = t_fs_meas;
    t_cal_active = false;
    cal_done = true;
  }
  if (cal_done) xQueueSend(g_cal_q, &cal, 0);

  // Zero (tare) completion
  if (t_tare_active && t_read_us >= t_tare_end_us) {
    TareResult tr;
    tr.axis = t_cfg.primary_axis % 3;
    tr.samples = t_tare_n;
    t_tare_active = false;
    if (t_tare_n < 20) {
      tr.reason = "too few samples";
    } else {
      const double m = t_tare_sum / t_tare_n;
      const double var = t_tare_sumsq / t_tare_n - m * m;
      tr.value = float(m);
      tr.stdev = float(sqrt(var > 0 ? var : 0));
      if (tr.stdev > cfg::TARE_MAX_STD) {
        tr.reason = "sensor moving - hold it still and press again";
      } else {
        tr.ok = true;
        t_cfg.tare[tr.axis] = tr.value;
        t_base.reset();  // restart the baseline at the new level: no false impact
        t_win.reset();
        publishConfig();
      }
    }
    xQueueSend(g_tare_q, &tr, 0);
  }

  // Publish snapshot
  uint32_t flags = ST_SENSOR_OK | (t_running ? uint32_t(ST_RUNNING) : 0u);
  if (t_fs_meas > 0 && fabsf(t_fs_meas - t_fs_nom) / t_fs_nom > cfg::RATE_DEVIATION_LIMIT) flags |= ST_RATE_DEV;
  if (t_dup_ratio > cfg::DUPLICATE_RATIO_LIMIT) flags |= ST_DUPLICATES;
  if (SENSOR_PROFILES[t_cfg.profile].alias_risk) flags |= ST_ALIAS_RISK;

  portENTER_CRITICAL(&g_mux);
  g_snap.idx = t_idx - 1;
  g_snap.t_us = t_us;
  for (int i = 0; i < 3; ++i) {
    g_snap.a[i] = a[i];
    g_snap.d[i] = d[i];
  }
  g_snap.amag = amag;
  g_snap.dmag = dmag;
  if (win_done) g_snap.win = win;
  if (batch_peak > g_snap.peak_interval) g_snap.peak_interval = batch_peak;
  g_snap.test = t_test;
  g_snap.impacts = t_det.count();
  g_snap.impact_active = t_det.active();
  g_snap.fs_meas = t_fs_meas;
  g_snap.dup_ratio = t_dup_ratio;
  g_snap.flags = (g_snap.flags & ~(ST_SENSOR_OK | ST_RUNNING | ST_RATE_DEV | ST_DUPLICATES |
                                   ST_ALIAS_RISK | ST_RECOVERING)) |
                 flags | t_flags_latched;
  t_flags_latched = 0;
  g_fft_written += frames;
  g_fft_contiguous += frames;
  portEXIT_CRITICAL(&g_mux);
}

void pollFifo() {
  uint8_t st = 0;
  uint16_t count = 0;
  if (g_mpu.readIntStatus(st) != MpuError::OK || g_mpu.readFifoCount(count) != MpuError::OK) {
    onI2cError();
    return;
  }
  const int64_t now = esp_timer_get_time();

  // Overflow: FIFO_MODE = 1 stops writing when full, so samples were lost and
  // frame alignment can no longer be trusted. Discard, reset and mark a gap.
  if ((st & mpu::INT_STATUS_FIFO_OFLOW) || count >= mpu::FIFO_SIZE_BYTES - 1) {
    if (g_mpu.resetFifo() != MpuError::OK) {
      onI2cError();
      return;
    }
    t_flags_latched |= ST_FIFO_OVF;
    portENTER_CRITICAL(&g_mux);
    g_snap.fifo_overflows++;
    portEXIT_CRITICAL(&g_mux);
    markGap(now);
    return;
  }

  const size_t frames = count / mpu::FIFO_FRAME_BYTES;
  if (frames == 0) return;
  if (g_mpu.readFifo(t_buf, frames * mpu::FIFO_FRAME_BYTES) != MpuError::OK) {
    onI2cError();
    return;
  }
  t_consec_err = 0;
  processBatch(t_buf, frames, now);
}

void restartStream() {
  // Drop untransmitted samples of the previous configuration.
  g_stream_tail.store(g_stream_head.load(std::memory_order_acquire), std::memory_order_release);
}

void handleCommand(const AcqCommand& c) {
  switch (c.type) {
    case AcqCmd::START:
      if (!t_running) {
        t_running = true;
        if (t_sensor_ok && g_mpu.resetFifo() != MpuError::OK) onI2cError();
        t_base.reset();
        markGap(esp_timer_get_time());
      }
      break;
    case AcqCmd::STOP:
      t_running = false;
      break;
    case AcqCmd::RESET_STATS:
      t_test.reset();
      t_det.reset();
      t_win.reset();
      portENTER_CRITICAL(&g_mux);
      g_snap.peak_interval = 0;
      g_snap.win = dsp::WindowResult();
      portEXIT_CRITICAL(&g_mux);
      break;
    case AcqCmd::SET_CONFIG: {
      const bool sensor_change = c.cfg.profile != t_cfg.profile || c.cfg.range != t_cfg.range;
      const bool stream_change = c.cfg.stream_raw != t_cfg.stream_raw;
      t_cfg = c.cfg;
      if (sensor_change) {
        // New scale factor / sample rate: re-initialise and restart all filters.
        if (initSensor()) {
          t_fs_meas = 0;
          t_test.reset();
          t_det.reset();
          markGap(esp_timer_get_time());
          clearFftRing();
        }
      } else {
        applyDspConfig();
      }
      publishConfig();
      if (sensor_change || stream_change) restartStream();
      break;
    }
    case AcqCmd::CALIBRATE:
      t_cal_active = true;
      t_cal_end_us = esp_timer_get_time() + int64_t(c.cal_ms) * 1000;
      t_cal_n = t_cal_clip = 0;
      for (int i = 0; i < 3; ++i) t_cal_sum[i] = t_cal_sumsq[i] = 0;
      break;
    case AcqCmd::TARE: {
      if (!t_sensor_ok || !t_running) {
        TareResult tr;
        tr.axis = t_cfg.primary_axis % 3;
        tr.reason = t_sensor_ok ? "acquisition stopped - send START" : "sensor not available";
        xQueueSend(g_tare_q, &tr, 0);
        break;
      }
      const int64_t now = esp_timer_get_time();
      t_tare_active = true;
      t_tare_start_us = now + int64_t(c.settle_ms) * 1000;
      t_tare_end_us = t_tare_start_us + int64_t(cfg::TARE_AVERAGE_MS) * 1000;
      t_tare_n = 0;
      t_tare_sum = t_tare_sumsq = 0;
      break;
    }
  }
}

void acqTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    AcqCommand cmd;
    while (xQueueReceive(g_cmd_q, &cmd, 0) == pdTRUE) handleCommand(cmd);

    if (!t_sensor_ok) {
      portENTER_CRITICAL(&g_mux);
      g_snap.flags = (g_snap.flags & ~(ST_SENSOR_OK | ST_RUNNING)) | ST_RECOVERING;
      portEXIT_CRITICAL(&g_mux);
      if (initSensor()) {
        markGap(esp_timer_get_time());
        clearFftRing();
        portENTER_CRITICAL(&g_mux);
        g_snap.flags = (g_snap.flags & ~ST_RECOVERING) | ST_SENSOR_OK;
        portEXIT_CRITICAL(&g_mux);
      } else {
        vTaskDelay(pdMS_TO_TICKS(cfg::RECOVERY_RETRY_MS));
      }
      last_wake = xTaskGetTickCount();
      continue;
    }

    if (t_running) {
      pollFifo();
    } else {
      portENTER_CRITICAL(&g_mux);
      g_snap.flags &= ~ST_RUNNING;
      portEXIT_CRITICAL(&g_mux);
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(cfg::ACQ_POLL_PERIOD_MS));
  }
}

}  // namespace

namespace acq {

bool start(const RuntimeConfig& initial) {
  t_cfg = initial;
  publishConfig();
  g_cmd_q = xQueueCreate(cfg::COMMAND_QUEUE_LEN, sizeof(AcqCommand));
  g_event_q = xQueueCreate(cfg::EVENT_QUEUE_LEN, sizeof(ImpactEvent));
  g_cal_q = xQueueCreate(2, sizeof(CalResult));
  g_tare_q = xQueueCreate(2, sizeof(TareResult));
  if (!g_cmd_q || !g_event_q || !g_cal_q || !g_tare_q) return false;
  return xTaskCreatePinnedToCore(acqTask, "acq", cfg::ACQ_TASK_STACK, nullptr,
                                 cfg::ACQ_TASK_PRIORITY, nullptr, cfg::ACQ_TASK_CORE) == pdPASS;
}

bool post(const AcqCommand& cmd) { return xQueueSend(g_cmd_q, &cmd, pdMS_TO_TICKS(10)) == pdTRUE; }

AcqSnapshot snapshot(bool consume) {
  portENTER_CRITICAL(&g_mux);
  AcqSnapshot s = g_snap;
  if (consume) {
    g_snap.peak_interval = 0;
    g_snap.flags &= ~(ST_FIFO_OVF | ST_I2C_ERR | ST_CLIP | ST_STREAM_DROP | ST_GAP);
  }
  portEXIT_CRITICAL(&g_mux);
  return s;
}

RuntimeConfig config(uint32_t& gen) {
  portENTER_CRITICAL(&g_mux);
  RuntimeConfig c = g_cfg;
  gen = g_gen;
  portEXIT_CRITICAL(&g_mux);
  return c;
}

size_t readStream(RawSample* dst, size_t max) {
  uint32_t tail = g_stream_tail.load(std::memory_order_relaxed);
  const uint32_t head = g_stream_head.load(std::memory_order_acquire);
  size_t n = 0;
  while (tail != head && n < max) {
    const RawSample& s = g_stream[tail % cfg::STREAM_RING_SAMPLES];
    if (n > 0 && (s.gen != dst[0].gen || s.idx != dst[n - 1].idx + 1)) break;
    dst[n++] = s;
    ++tail;
  }
  g_stream_tail.store(tail, std::memory_order_release);
  return n;
}

bool copyFftWindow(float* dst, size_t n) {
  if (n > cfg::FFT_MAX_N) return false;
  portENTER_CRITICAL(&g_mux);
  const bool ok = g_fft_contiguous >= n && g_fft_written >= n;
  if (ok) {
    const uint32_t end = g_snap.idx + 1;  // ring position after the newest sample
    for (size_t i = 0; i < n; ++i) dst[i] = g_fft_ring[(end - n + i) % cfg::FFT_RING];
  }
  portEXIT_CRITICAL(&g_mux);
  return ok;
}

bool popEvent(ImpactEvent& ev) { return xQueueReceive(g_event_q, &ev, 0) == pdTRUE; }
bool popCalibration(CalResult& cal) { return xQueueReceive(g_cal_q, &cal, 0) == pdTRUE; }
bool popTare(TareResult& t) { return xQueueReceive(g_tare_q, &t, 0) == pdTRUE; }

uint8_t sensorAddress() {
  portENTER_CRITICAL(&g_mux);
  const uint8_t a = g_addr;
  portEXIT_CRITICAL(&g_mux);
  return a;
}

uint8_t sensorWhoAmI() {
  portENTER_CRITICAL(&g_mux);
  const uint8_t w = g_who;
  portEXIT_CRITICAL(&g_mux);
  return w;
}

const char* initError() {
  portENTER_CRITICAL(&g_mux);
  const char* e = g_init_error;
  portEXIT_CRITICAL(&g_mux);
  return e;
}

bool takeInitChanged() {
  portENTER_CRITICAL(&g_mux);
  const bool c = g_init_changed;
  g_init_changed = false;
  portEXIT_CRITICAL(&g_mux);
  return c;
}

}  // namespace acq
