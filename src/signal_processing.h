// signal_processing.h — per-sample DSP building blocks.
//
// Terminology used throughout the project:
//   raw acceleration      a      : sensor output converted to m/s^2 (includes gravity)
//   baseline              b      : slowly varying part of a (gravity + static tilt),
//                                  estimated per axis with a first-order low-pass
//   dynamic acceleration  d = a-b: baseline-removed acceleration
//
// d is NOT a true gravity-free linear acceleration: if the sensor rotates
// faster than the baseline time constant, the change in the gravity
// projection appears in d. With a 6-axis IMU this could be corrected with an
// orientation filter; for a shock absorber whose top mount barely rotates, the
// baseline approach is adequate but must be validated (README, calibration).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dsp {

// First-order IIR low-pass per axis: b[n] = b[n-1] + alpha * (a[n] - b[n-1]),
// alpha = 1 - exp(-1 / (fs * tau)). Equivalent high-pass corner for d:
// fc = 1 / (2*pi*tau).
class BaselineFilter {
 public:
  void configure(float fs_hz, float tau_s);
  void reset() { initialised_ = false; }
  // Updates the baseline with sample a[3] and writes d = a - b to dyn[3].
  void update(const float a[3], float dyn[3]);
  const float* baseline() const { return b_; }

 private:
  float alpha_ = 0.002f;
  float b_[3] = {0, 0, 0};
  bool initialised_ = false;
};

// Non-overlapping block window: RMS and peak of the vibration value v, and
// peak-to-peak of the signed primary-axis dynamic acceleration. The firmware
// feeds v = |d_primary| (Z by default), see acquisition.cpp.
struct WindowResult {
  float rms = 0;       // sqrt(mean(v^2))                   [m/s^2]
  float peak = 0;      // max v                             [m/s^2]
  float p2p = 0;       // max(d_primary) - min(d_primary)   [m/s^2]
  uint32_t samples = 0;
};

class WindowStats {
 public:
  void configure(uint32_t window_samples);
  void reset();
  // Returns true when a window completes; the result is written to `out`.
  bool add(float dmag, float dprimary, WindowResult& out);

 private:
  uint32_t n_target_ = 200;
  uint32_t n_ = 0;
  double sumsq_ = 0;
  float peak_ = 0, pmax_ = 0, pmin_ = 0;
};

// Whole-test accumulators (since the last RESET command).
struct TestStats {
  uint32_t samples = 0;
  double sumsq = 0;               // sum of v^2 (v = |d_primary|)
  float max_dyn = 0;              // max v
  float max_raw[3] = {0, 0, 0};   // max raw acceleration per axis
  float min_raw[3] = {0, 0, 0};   // min raw acceleration per axis
  float max_dprimary = 0;         // max signed primary-axis d
  float min_dprimary = 0;         // min signed primary-axis d
  uint32_t clipped = 0;           // samples with any axis at full scale

  void reset() { *this = TestStats(); }
  void add(const float a[3], float dmag, float dprimary, bool clip);
  float rms() const;
  float p2p() const { return samples ? max_dprimary - min_dprimary : 0.f; }
};

// ---------------------------------------------------------------------------
// Frequency analysis
// ---------------------------------------------------------------------------

// In-place iterative radix-2 complex FFT. n must be a power of two.
void fft(float* re, float* im, size_t n);

struct SpectrumPeak {
  float freq_hz = 0;      // dominant frequency (parabolic interpolation between bins)
  float amp = 0;          // single-sided amplitude of the peak [m/s^2]
  float noise_floor = 0;  // median single-sided amplitude in the search band
  float df_hz = 0;        // bin spacing fs / n
  bool valid = false;     // peak is >= 12 dB above the band median and inside the valid band
};

// Hann-windowed amplitude spectrum of x[0..n) sampled at fs; the dominant peak
// is searched only in [fmin, fmax]. `re`/`im` are scratch buffers of length n.
// Returns valid = false if fmax <= fmin (e.g. profile has no validated band).
SpectrumPeak dominantFrequency(const float* x, size_t n, float fs, float fmin, float fmax,
                               float* re, float* im);

}  // namespace dsp
