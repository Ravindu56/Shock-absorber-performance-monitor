// config.h — compile-time constants and runtime-configurable parameters.
//
// Every threshold, window length and timing value used by the firmware lives
// here. Runtime-adjustable values are grouped in RuntimeConfig and can be
// changed over USB Serial with "SET <key> <value>" (see serial_protocol.cpp).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cfg {

// ---------------------------------------------------------------------------
// Hardware wiring (fixed by the project specification — do not change)
//   MPU6500 VCC -> ESP32 3.3 V
//   MPU6500 GND -> ESP32 GND
//   MPU6500 SDA -> ESP32 GPIO 21 (D21)
//   MPU6500 SCL -> ESP32 GPIO 22 (D22)
// The MPU6500 INT pin is NOT connected, so acquisition uses FIFO polling.
// ---------------------------------------------------------------------------
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;
constexpr uint32_t I2C_FREQ_HZ = 400000;   // MPU6500 I2C fast-mode maximum
constexpr uint16_t I2C_TIMEOUT_MS = 10;    // per-transaction timeout

// Zero (tare) push button: GPIO4 -> button -> GND, internal pull-up (pressed = LOW).
// The on-board LED (GPIO2 on the DOIT DevKit V1) shows zeroing progress.
constexpr int PIN_ZERO_BUTTON = 4;
constexpr int PIN_STATUS_LED = 2;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;     // level must be stable this long
constexpr uint32_t TARE_SETTLE_MS = 500;        // wait after release (finger vibration dies out)
constexpr uint32_t TARE_AVERAGE_MS = 1000;      // averaging window
constexpr float TARE_MAX_STD = 0.3f;            // m/s^2; above this the sensor was moving -> refuse

constexpr uint8_t MPU_ADDR_AD0_LOW = 0x68;
constexpr uint8_t MPU_ADDR_AD0_HIGH = 0x69;

// ---------------------------------------------------------------------------
// Serial link
// ---------------------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 921600;   // CP2102 / CH340 on DevKit V1 support this
constexpr size_t SERIAL_TX_BUFFER = 8192;  // large TX buffer so bursts don't block the reporter
constexpr size_t SERIAL_RX_BUFFER = 512;

// ---------------------------------------------------------------------------
// Physics
// ---------------------------------------------------------------------------
constexpr float G_TO_MS2 = 9.80665f;       // a(m/s^2) = a(g) * 9.80665

// ---------------------------------------------------------------------------
// Task layout
// ---------------------------------------------------------------------------
constexpr uint32_t ACQ_POLL_PERIOD_MS = 2;     // FIFO poll period (FIFO holds ~85 ms at 1 kHz)
constexpr int ACQ_TASK_CORE = 1;
constexpr int ACQ_TASK_PRIORITY = 5;
constexpr uint32_t ACQ_TASK_STACK = 6144;
constexpr int REPORT_TASK_CORE = 0;
constexpr int REPORT_TASK_PRIORITY = 2;
constexpr uint32_t REPORT_TASK_STACK = 12288;  // FFT scratch lives in static buffers, not stack
constexpr uint32_t REPORT_LOOP_MS = 5;

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------
constexpr size_t STREAM_RING_SAMPLES = 2048;   // raw samples awaiting transmission (~2 s at 1 kHz)
constexpr size_t STREAM_BLOCK_SAMPLES = 50;    // samples per "R" line
constexpr size_t FFT_RING = 4096;              // ring of primary-axis samples for the FFT
constexpr size_t FFT_MAX_N = FFT_RING / 2;     // largest FFT (half the ring: reader never races the writer)
constexpr size_t EVENT_QUEUE_LEN = 16;
constexpr size_t COMMAND_QUEUE_LEN = 8;

// ---------------------------------------------------------------------------
// Error handling
// ---------------------------------------------------------------------------
constexpr uint32_t I2C_ERRORS_BEFORE_RECOVERY = 10;  // consecutive failures -> bus + sensor re-init
constexpr uint32_t RECOVERY_RETRY_MS = 1000;
constexpr float RATE_DEVIATION_LIMIT = 0.03f;        // flag if |fs_meas - fs_nom| / fs_nom > 3 %
constexpr float DUPLICATE_RATIO_LIMIT = 0.20f;       // flag if >20 % of samples repeat exactly

}  // namespace cfg

// ---------------------------------------------------------------------------
// Sensor profiles.
//
// Each profile is a validated combination of MPU6500 register settings. The
// "valid_max_hz" field is the highest frequency the firmware/dashboard will
// treat as a validated spectral measurement for that profile; it is set below
// the DLPF -3 dB corner to leave margin for filter roll-off and residual
// aliasing (the on-chip DLPF is not a brick-wall anti-aliasing filter).
//
// Why there is no 4 kHz profile: with ACCEL_FCHOICE_B = 1 the accelerometer
// runs at 4 kHz with a 1.13 kHz bandwidth, but the FIFO is then written at the
// 8 kHz internal rate (accel samples duplicated). 8 kHz x 6 bytes = 48 kB/s,
// which exceeds what a 400 kHz I2C bus can transfer (~40 kB/s raw). Polling
// data registers at 4 kHz would need ~90 % bus occupancy with no timing margin.
// See README.md, "Bandwidth and frequency limits".
// ---------------------------------------------------------------------------
struct SensorProfile {
  const char* name;
  uint8_t smplrt_div;      // SMPLRT_DIV (0x19): ODR = 1 kHz / (1 + div)
  uint8_t gyro_dlpf_cfg;   // CONFIG (0x1A) DLPF_CFG: 1..6 gives 1 kHz internal rate so SMPLRT_DIV applies
  uint8_t a_dlpf_cfg;      // ACCEL_CONFIG2 (0x1D) A_DLPF_CFG
  float odr_hz;            // nominal output / FIFO rate
  float bw_hz;             // datasheet accel -3 dB bandwidth for this A_DLPF_CFG
  float valid_max_hz;      // highest frequency reported as validated (0 = none)
  bool alias_risk;         // true if Nyquist is not well above the DLPF corner
};

// Indices into SENSOR_PROFILES
enum ProfileId : uint8_t {
  PROFILE_STD_1K_218 = 0,   // default: 1 kHz, 218 Hz BW
  PROFILE_LN_1K_99 = 1,     // low noise, strong anti-aliasing
  PROFILE_LN_500_45 = 2,    // very low noise, slow dynamics only
  PROFILE_EXP_1K_420 = 3,   // experimental: 420 Hz BW at 1 kHz (aliasing likely)
  PROFILE_COUNT
};

constexpr SensorProfile SENSOR_PROFILES[PROFILE_COUNT] = {
    // name           div gdlpf adlpf  odr     bw      valid  alias
    {"STD_1K_218",     0,  1,    1,   1000.f, 218.1f, 200.f, false},
    {"LN_1K_99",       0,  1,    2,   1000.f,  99.0f,  90.f, false},
    {"LN_500_45",      1,  1,    3,    500.f,  44.8f,  40.f, false},
    {"EXP_1K_420",     0,  1,    7,   1000.f, 420.0f,   0.f, true},
};

// Accelerometer full-scale range: ACCEL_CONFIG (0x1C) ACCEL_FS_SEL[4:3]
enum RangeId : uint8_t { RANGE_2G = 0, RANGE_4G = 1, RANGE_8G = 2, RANGE_16G = 3 };
constexpr uint8_t RANGE_G[4] = {2, 4, 8, 16};
constexpr float RANGE_LSB_PER_G[4] = {16384.f, 8192.f, 4096.f, 2048.f};

// ---------------------------------------------------------------------------
// Runtime-configurable parameters (defaults shown). Changing profile or range
// re-initialises the sensor; other changes take effect immediately.
// ---------------------------------------------------------------------------
struct RuntimeConfig {
  uint8_t profile = PROFILE_STD_1K_218;
  uint8_t range = RANGE_16G;          // impacts can be large; check the clip counter

  // Signal processing
  float baseline_tau_s = 0.5f;        // EMA baseline time constant (high-pass corner ~ 1/(2*pi*tau) = 0.32 Hz)
  uint32_t rms_window_ms = 200;       // block length for RMS / P2P windows
  uint8_t primary_axis = 2;           // 0=X 1=Y 2=Z; axis aligned with shock stroke (signed P2P, FFT)

  // Impact detector (all values in m/s^2 / ms of dynamic acceleration magnitude)
  float impact_threshold = 9.81f;     // trigger level (1 g of dynamic acceleration)
  float impact_hysteresis = 2.0f;     // release level = threshold - hysteresis
  uint32_t impact_persist_samples = 2;// consecutive samples above threshold to trigger
  uint32_t event_window_ms = 300;     // maximum event length (peak tracked inside it)
  uint32_t release_hold_ms = 20;      // must stay below release level this long to end event
  uint32_t refractory_ms = 200;       // dead time after event end

  // Reporting
  uint32_t report_hz = 20;            // "S" status lines per second
  bool stream_raw = true;             // send full-rate "R" raw sample blocks
  uint32_t fft_n = 1024;              // FFT length (power of two, 256..FFT_MAX_N); df = fs / fft_n
  uint32_t fft_interval_ms = 1000;    // how often to compute the on-device FFT
  float fft_fmin_hz = 2.0f;           // ignore bins below this when finding the dominant peak

  // Accelerometer calibration (from the 6-position procedure, README "Calibration"):
  //   a_cal[i] = (a_datasheet[i] - offset[i]) * scale[i]
  // a_datasheet uses the nominal LSB/g. Offsets are in m/s^2, so they stay valid
  // when the range changes.
  float offset[3] = {0.f, 0.f, 0.f};
  float scale[3] = {1.f, 1.f, 1.f};

  // Zero (tare) offset, subtracted after calibration: a_out = a_cal - tare.
  // Set by the GPIO4 button or the TARE command so the monitored axis reads 0
  // at rest (gravity removed). Kept separate from the calibration above so the
  // stationary |g| check stays meaningful.
  float tare[3] = {0.f, 0.f, 0.f};
};

namespace cfg {
// NVS (flash) storage of RuntimeConfig: "SAVE" writes it, "DEFAULTS" erases it.
constexpr const char* NVS_NAMESPACE = "shockmon";
constexpr const char* NVS_KEY = "cfg";
constexpr uint32_t NVS_MAGIC = 0x53484B32;  // "SHK2" — bump if RuntimeConfig layout changes
}  // namespace cfg
