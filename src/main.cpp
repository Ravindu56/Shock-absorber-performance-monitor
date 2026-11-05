// main.cpp — ESP32 + MPU6500 shock absorber performance monitor.
//
// Two FreeRTOS tasks:
//   acq    (core 1, high priority)  FIFO polling, DSP, impact detection — see acquisition.cpp
//   report (core 0, low priority)   Serial commands, status/event/raw/FFT output
//
// The Arduino loop task is deleted after setup(); all work happens in the tasks.
//
// Zero button (GPIO4 to GND, internal pull-up): press and release, keep the
// sensor still; the monitored axis (Z) is averaged for 1 s and that value is
// subtracted from then on, so Z reads 0 at rest (gravity removed). The on-board
// LED is lit while zeroing; 3 short blinks = done, 10 fast blinks = refused.
#include <Arduino.h>

#include "acquisition.h"
#include "config.h"
#include "config_store.h"
#include "serial_protocol.h"
#include "signal_processing.h"

namespace {

// FFT scratch buffers (static: too large for the task stack).
float g_fft_x[cfg::FFT_MAX_N];
float g_fft_re[cfg::FFT_MAX_N];
float g_fft_im[cfg::FFT_MAX_N];

// Configuration requested by the host; only the report task modifies it.
RuntimeConfig g_desired;

// Raw stream block being assembled for the next "R" line.
RawSample g_blk[cfg::STREAM_BLOCK_SAMPLES];
size_t g_blk_n = 0;
uint32_t g_blk_t0 = 0;
constexpr uint32_t STREAM_FLUSH_MS = 100;  // send a partial block if it is older than this

void flushBlock() {
  if (g_blk_n == 0) return;
  proto::sendRaw(g_blk, g_blk_n);
  g_blk_n = 0;
}

// Move samples from the acquisition ring into "R" lines. A block only ever
// holds consecutive samples of one configuration generation, so the host can
// convert counts with the right scale and detect gaps from first_idx.
void pumpStream() {
  RawSample tmp[cfg::STREAM_BLOCK_SAMPLES];
  for (;;) {
    const size_t n = acq::readStream(tmp, cfg::STREAM_BLOCK_SAMPLES - g_blk_n);
    if (n == 0) break;
    if (g_blk_n > 0 && (tmp[0].gen != g_blk[0].gen || tmp[0].idx != g_blk[g_blk_n - 1].idx + 1)) {
      flushBlock();
    }
    if (g_blk_n == 0) g_blk_t0 = millis();
    memcpy(&g_blk[g_blk_n], tmp, n * sizeof(RawSample));
    g_blk_n += n;
    if (g_blk_n >= cfg::STREAM_BLOCK_SAMPLES) flushBlock();
  }
  if (g_blk_n > 0 && millis() - g_blk_t0 >= STREAM_FLUSH_MS) flushBlock();
}

// ------------------------------------------------------------ zero button / LED
enum class LedMode : uint8_t { OFF, ON, BLINK };
LedMode g_led_mode = LedMode::OFF;
uint32_t g_led_t0 = 0, g_led_period = 0, g_led_toggles = 0;

void ledSet(LedMode m, uint32_t period_ms = 0, uint32_t blinks = 0) {
  g_led_mode = m;
  g_led_t0 = millis();
  g_led_period = period_ms;
  g_led_toggles = blinks * 2;
  digitalWrite(cfg::PIN_STATUS_LED, m == LedMode::OFF ? LOW : HIGH);
}

void ledUpdate() {
  if (g_led_mode != LedMode::BLINK || millis() - g_led_t0 < g_led_period) return;
  g_led_t0 = millis();
  if (g_led_toggles == 0) {
    ledSet(LedMode::OFF);
    return;
  }
  --g_led_toggles;
  digitalWrite(cfg::PIN_STATUS_LED, !digitalRead(cfg::PIN_STATUS_LED));
}

bool requestTare(const char* source) {
  AcqCommand ac{};
  ac.type = AcqCmd::TARE;
  ac.settle_ms = cfg::TARE_SETTLE_MS;
  if (!acq::post(ac)) {
    proto::sendAck("TARE", false, "acquisition command queue full");
    return false;
  }
  char msg[96];
  snprintf(msg, sizeof(msg), "zeroing (%s) - keep the sensor still for %u ms", source,
           unsigned(cfg::TARE_SETTLE_MS + cfg::TARE_AVERAGE_MS));
  proto::sendAck("TARE", true, msg);
  ledSet(LedMode::ON);
  return true;
}

// Debounced button: a tare is requested on RELEASE, so the averaging never
// includes the finger pressing on the assembly.
void pollButton() {
  static bool stable_pressed = false, last_raw = false;
  static uint32_t last_change = 0;
  const bool raw = digitalRead(cfg::PIN_ZERO_BUTTON) == LOW;
  const uint32_t now = millis();
  if (raw != last_raw) {
    last_raw = raw;
    last_change = now;
  }
  if (raw != stable_pressed && now - last_change >= cfg::BUTTON_DEBOUNCE_MS) {
    stable_pressed = raw;
    if (stable_pressed) {
      ledSet(LedMode::ON);  // immediate feedback while held
    } else {
      requestTare("button");
    }
  }
}

void handleTareResults() {
  TareResult tr;
  while (acq::popTare(tr)) {
    char msg[128];
    if (!tr.ok) {
      snprintf(msg, sizeof(msg), "zeroing refused: %s (std %.3f m/s^2)", tr.reason, double(tr.stdev));
      proto::sendAck("TARE", false, msg);
      ledSet(LedMode::BLINK, 60, 10);
      continue;
    }
    g_desired.tare[tr.axis] = tr.value;  // keep later SET commands from undoing the zero
    uint32_t gen;
    const bool saved = store::save(acq::config(gen));
    snprintf(msg, sizeof(msg), "%c axis zeroed: tare %.4f m/s^2 (%.4f g), std %.4f, %u samples%s",
             "XYZ"[tr.axis], double(tr.value), double(tr.value / cfg::G_TO_MS2), double(tr.stdev),
             unsigned(tr.samples), saved ? ", saved to flash" : ", NVS save FAILED");
    proto::sendAck("TARE", true, msg);
    ledSet(LedMode::BLINK, 120, 3);
  }
}

void postOrError(const AcqCommand& c, const char* name, const char* okmsg) {
  if (acq::post(c)) {
    proto::sendAck(name, true, okmsg);
  } else {
    proto::sendAck(name, false, "acquisition command queue full");
  }
}

void handleSerial() {
  char line[128];
  while (proto::readLine(line, sizeof(line))) {
    RuntimeConfig trial = g_desired;  // committed only if the SET reaches the acq task
    const proto::ParsedCommand pc = proto::parseCommand(line, trial);
    AcqCommand ac{};
    switch (pc.cmd) {
      case proto::Command::NONE:
        break;
      case proto::Command::PING:
        proto::sendAck(pc.name, true, "PONG");
        break;
      case proto::Command::HELLO: {
        uint32_t gen;
        const RuntimeConfig c = acq::config(gen);
        proto::sendHello();
        proto::sendHeaders();
        proto::sendConfig(c, gen);
        break;
      }
      case proto::Command::GETCFG: {
        uint32_t gen;
        const RuntimeConfig c = acq::config(gen);
        proto::sendConfig(c, gen);
        break;
      }
      case proto::Command::START:
        ac.type = AcqCmd::START;
        postOrError(ac, pc.name, "acquisition running");
        break;
      case proto::Command::STOP:
        ac.type = AcqCmd::STOP;
        postOrError(ac, pc.name, "acquisition stopped");
        break;
      case proto::Command::RESET:
        ac.type = AcqCmd::RESET_STATS;
        postOrError(ac, pc.name, "test statistics and impact count cleared");
        break;
      case proto::Command::CAL: {
        ac.type = AcqCmd::CALIBRATE;
        ac.cal_ms = pc.cal_ms;
        char msg[64];
        snprintf(msg, sizeof(msg), "collecting %u ms - keep the sensor still", unsigned(pc.cal_ms));
        postOrError(ac, pc.name, msg);
        break;
      }
      case proto::Command::SET:
        ac.type = AcqCmd::SET_CONFIG;
        ac.cfg = trial;
        if (acq::post(ac)) {
          g_desired = trial;
          proto::sendAck(pc.name, true, "applied");  // a CFG line follows when the task applies it
        } else {
          proto::sendAck(pc.name, false, "acquisition command queue full");
        }
        break;
      case proto::Command::SAVE: {
        uint32_t gen;
        const RuntimeConfig c = acq::config(gen);
        const bool ok = store::save(c);
        proto::sendAck(pc.name, ok, ok ? "configuration saved to flash" : "NVS write failed");
        break;
      }
      case proto::Command::DEFAULTS: {
        const bool ok = store::erase();
        g_desired = RuntimeConfig();
        ac.type = AcqCmd::SET_CONFIG;
        ac.cfg = g_desired;
        postOrError(ac, pc.name, ok ? "defaults restored, flash copy erased" : "defaults restored (NVS erase failed)");
        break;
      }
      case proto::Command::TARE:
        requestTare("command");
        break;
      case proto::Command::INVALID:
        proto::sendAck(pc.name[0] ? pc.name : "?", false, pc.error);
        break;
    }
  }
}

void runFft(const RuntimeConfig& c, const AcqSnapshot& s) {
  const SensorProfile& p = SENSOR_PROFILES[c.profile];
  const size_t n = c.fft_n;
  if (!acq::copyFftWindow(g_fft_x, n)) return;  // not enough gap-free data yet
  // Use the measured rate (ESP32 crystal) when available: the MPU6500 runs from
  // its internal oscillator when the gyro is off, so the nominal rate can be off
  // by a few percent, which would shift every reported frequency by the same %.
  const float fs = s.fs_meas > 0 ? s.fs_meas : p.odr_hz;
  const float fmax = p.valid_max_hz;
  const dsp::SpectrumPeak pk = dsp::dominantFrequency(g_fft_x, n, fs, c.fft_fmin_hz, fmax,
                                                      g_fft_re, g_fft_im);
  // Only call it a measurement when the band is validated and the rate verified.
  const bool rate_ok = s.fs_meas > 0 && !(s.flags & ST_RATE_DEV);
  const bool valid = pk.valid && fmax > 0 && !p.alias_risk && rate_ok;
  proto::sendSpectrum(s.idx, n, fs, c.fft_fmin_hz, fmax, pk, valid);
}

void reportTask(void*) {
  uint32_t last_status = 0, last_test = 0, last_fft = 0;
  uint32_t sent_gen = UINT32_MAX;

  // From here on this task is the only Serial writer (emitf uses one static buffer).
  proto::sendHello();
  proto::sendHeaders();

  for (;;) {
    handleSerial();

    uint32_t gen;
    const RuntimeConfig c = acq::config(gen);
    if (gen != sent_gen) {
      flushBlock();  // samples of the old generation go out before the new CFG
      proto::sendConfig(c, gen);
      sent_gen = gen;
    }
    if (acq::takeInitChanged()) {
      proto::sendHello();
      const char* err = acq::initError();
      if (strcmp(err, "OK") != 0) {
        char msg[96];
        snprintf(msg, sizeof(msg), "sensor init failed (%s) - check wiring SDA=%d SCL=%d, 3.3 V, GND; retrying",
                 err, cfg::PIN_SDA, cfg::PIN_SCL);
        proto::sendError("SENSOR", msg);
      }
    }

    pumpStream();

    ImpactEvent ev;
    while (acq::popEvent(ev)) proto::sendEvent(ev);
    CalResult cal;
    while (acq::popCalibration(cal)) proto::sendCalibration(cal);

    pollButton();
    handleTareResults();
    ledUpdate();

    const uint32_t now = millis();
    if (now - last_status >= 1000 / c.report_hz) {
      last_status = now;
      const AcqSnapshot s = acq::snapshot(true);
      proto::sendStatus(s);
      if (now - last_test >= 1000) {
        last_test = now;
        proto::sendTestStats(s);
      }
      if (now - last_fft >= c.fft_interval_ms && (s.flags & ST_RUNNING)) {
        last_fft = now;
        runFft(c, s);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(cfg::REPORT_LOOP_MS));
  }
}

}  // namespace

void setup() {
  Serial.setRxBufferSize(cfg::SERIAL_RX_BUFFER);
  Serial.setTxBufferSize(cfg::SERIAL_TX_BUFFER);
  Serial.begin(cfg::SERIAL_BAUD);
  delay(50);
  proto::sendLog("shock absorber monitor booting");

  pinMode(cfg::PIN_ZERO_BUTTON, INPUT_PULLUP);
  pinMode(cfg::PIN_STATUS_LED, OUTPUT);
  digitalWrite(cfg::PIN_STATUS_LED, LOW);

  RuntimeConfig initial;
  if (store::load(initial)) {
    proto::sendLog("loaded saved configuration from flash");
  } else {
    proto::sendLog("no saved configuration - using defaults");
  }
  g_desired = initial;

  if (!acq::start(initial)) {
    // Out of memory at boot: nothing useful can run. Report and restart.
    for (int i = 0; i < 5; ++i) {
      proto::sendError("BOOT", "failed to create acquisition task/queues - restarting");
      delay(1000);
    }
    ESP.restart();
  }
  if (xTaskCreatePinnedToCore(reportTask, "report", cfg::REPORT_TASK_STACK, nullptr,
                              cfg::REPORT_TASK_PRIORITY, nullptr, cfg::REPORT_TASK_CORE) != pdPASS) {
    proto::sendError("BOOT", "failed to create report task - restarting");
    delay(1000);
    ESP.restart();
  }
  // No Serial output from here: the report task owns Serial now.
}

void loop() {
  vTaskDelete(nullptr);  // everything runs in the acq/report tasks
}
