// acquisition.h — high-priority sensor task: FIFO polling, timestamping,
// per-sample DSP and impact detection.
//
// The acquisition task is the only code that touches the I2C bus. It shares
// results with the (lower priority) reporting task through:
//   * a lock-protected snapshot of the latest values and statistics,
//   * a single-producer/single-consumer ring of raw samples (for streaming),
//   * a ring of primary-axis dynamic acceleration (for the FFT),
//   * FreeRTOS queues for impact events and calibration results.
// Commands flow the other way through a FreeRTOS queue and are applied
// between FIFO reads, so the sampling loop never waits on Serial.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "config.h"
#include "impact_detector.h"
#include "signal_processing.h"

// Status flag bits reported in every "S" line. Bits marked (latched) are set
// when the condition occurred at any time since the previous status line.
enum StatusFlag : uint32_t {
  ST_SENSOR_OK = 1u << 0,    // sensor initialised and responding
  ST_RUNNING = 1u << 1,      // acquisition started (START) and not stopped
  ST_FIFO_OVF = 1u << 2,     // (latched) FIFO overflowed, samples lost
  ST_I2C_ERR = 1u << 3,      // (latched) I2C transaction failed
  ST_CLIP = 1u << 4,         // (latched) a sample hit full scale on some axis
  ST_RATE_DEV = 1u << 5,     // measured rate deviates > 3 % from nominal
  ST_STREAM_DROP = 1u << 6,  // (latched) raw stream ring full, samples not transmitted
  ST_ALIAS_RISK = 1u << 7,   // active profile has no validated spectral band
  ST_RECOVERING = 1u << 8,   // bus/sensor recovery in progress
  ST_DUPLICATES = 1u << 9,   // > 20 % of consecutive samples identical (sensor not updating?)
  ST_GAP = 1u << 10,         // (latched) sample index jumped (overflow/recovery/restart)
};

struct RawSample {
  int16_t x, y, z;  // raw sensor counts
  uint8_t gen;      // configuration generation (scale factor) of this sample
  uint8_t pad;
  uint32_t idx;     // global sample index
};

struct AcqSnapshot {
  uint32_t idx = 0;         // index of the latest processed sample
  int64_t t_us = 0;         // ESP32 time estimate of that sample
  float a[3] = {0, 0, 0};   // raw acceleration [m/s^2]
  float amag = 0;           // |a| [m/s^2]
  float d[3] = {0, 0, 0};   // dynamic acceleration [m/s^2]
  float dmag = 0;           // |d| [m/s^2]
  dsp::WindowResult win;    // latest completed RMS/P2P window (primary axis)
  float peak_interval = 0;  // max |d_primary| since the previous status line
  dsp::TestStats test;
  uint32_t impacts = 0;
  bool impact_active = false;
  float fs_meas = 0;        // measured sample rate [Hz] (0 until the first 1 s window)
  float dup_ratio = 0;
  uint32_t flags = 0;
  uint32_t fifo_overflows = 0, i2c_errors = 0, stream_drops = 0, gaps = 0, recoveries = 0;
  uint32_t events_dropped = 0;
};

struct CalResult {
  uint32_t samples = 0;
  float mean[3] = {0, 0, 0};  // m/s^2, UNCALIBRATED (datasheet sensitivity, no offset/scale)
  float stdev[3] = {0, 0, 0}; // m/s^2 (noise / stability when stationary)
  float mag_mean = 0;         // |mean| [m/s^2], expect ~9.81 when stationary
  float roll_deg = 0, pitch_deg = 0;
  uint32_t clipped = 0;
  float fs_meas = 0;
};

// Result of a zero (tare) request.
struct TareResult {
  bool ok = false;
  uint8_t axis = 2;
  float value = 0;     // new tare offset [m/s^2] (the axis reading that becomes 0)
  float stdev = 0;     // m/s^2 during averaging
  uint32_t samples = 0;
  const char* reason = "";  // failure reason (static string)
};

enum class AcqCmd : uint8_t { START, STOP, RESET_STATS, SET_CONFIG, CALIBRATE, TARE };

struct AcqCommand {
  AcqCmd type;
  RuntimeConfig cfg;     // SET_CONFIG
  uint32_t cal_ms = 0;   // CALIBRATE
  uint32_t settle_ms = 0;  // TARE: delay before averaging starts
};

namespace acq {

// Create the acquisition task. Sensor initialisation happens inside the task
// (with retries), so failures are reported through status flags / initError().
bool start(const RuntimeConfig& initial);

bool post(const AcqCommand& cmd);

// Copy of the latest state. With consume = true the latched flags and the
// interval peak are cleared (the reporting task does this once per "S" line).
AcqSnapshot snapshot(bool consume);

// Active configuration and its generation counter (incremented on every change).
RuntimeConfig config(uint32_t& gen);

// Pop up to `max` raw samples that share one generation and have consecutive
// indices. Returns the number copied.
size_t readStream(RawSample* dst, size_t max);

// Copy the most recent n primary-axis dynamic samples (oldest first).
// Returns false if fewer than n contiguous gap-free samples are available.
bool copyFftWindow(float* dst, size_t n);

bool popEvent(ImpactEvent& ev);
bool popCalibration(CalResult& cal);
bool popTare(TareResult& t);

// Sensor identity / last initialisation error (for the HELLO / ERR lines).
uint8_t sensorAddress();
uint8_t sensorWhoAmI();
const char* initError();  // "OK" when initialised
bool takeInitChanged();   // true once after each init success/failure transition

}  // namespace acq
