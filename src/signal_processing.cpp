#include "signal_processing.h"

#include <math.h>

#include <algorithm>

namespace dsp {

void BaselineFilter::configure(float fs_hz, float tau_s) {
  if (fs_hz <= 0 || tau_s <= 0) {
    alpha_ = 1.f;
    return;
  }
  alpha_ = 1.f - expf(-1.f / (fs_hz * tau_s));
}

void BaselineFilter::update(const float a[3], float dyn[3]) {
  if (!initialised_) {
    // Start the baseline at the first sample so d starts at zero instead of
    // ringing for several time constants from b = 0.
    for (int i = 0; i < 3; ++i) b_[i] = a[i];
    initialised_ = true;
  }
  for (int i = 0; i < 3; ++i) {
    b_[i] += alpha_ * (a[i] - b_[i]);
    dyn[i] = a[i] - b_[i];
  }
}

void WindowStats::configure(uint32_t window_samples) {
  n_target_ = window_samples < 2 ? 2 : window_samples;
  reset();
}

void WindowStats::reset() {
  n_ = 0;
  sumsq_ = 0;
  peak_ = pmax_ = pmin_ = 0;
}

bool WindowStats::add(float dmag, float dprimary, WindowResult& out) {
  if (n_ == 0) {
    pmax_ = pmin_ = dprimary;
    peak_ = dmag;
  }
  sumsq_ += double(dmag) * dmag;
  if (dmag > peak_) peak_ = dmag;
  if (dprimary > pmax_) pmax_ = dprimary;
  if (dprimary < pmin_) pmin_ = dprimary;
  if (++n_ < n_target_) return false;

  out.rms = sqrtf(float(sumsq_ / n_));
  out.peak = peak_;
  out.p2p = pmax_ - pmin_;
  out.samples = n_;
  reset();
  return true;
}

void TestStats::add(const float a[3], float dmag, float dprimary, bool clip) {
  if (samples == 0) {
    for (int i = 0; i < 3; ++i) max_raw[i] = min_raw[i] = a[i];
    max_dprimary = min_dprimary = dprimary;
    max_dyn = dmag;
  }
  ++samples;
  sumsq += double(dmag) * dmag;
  if (dmag > max_dyn) max_dyn = dmag;
  for (int i = 0; i < 3; ++i) {
    if (a[i] > max_raw[i]) max_raw[i] = a[i];
    if (a[i] < min_raw[i]) min_raw[i] = a[i];
  }
  if (dprimary > max_dprimary) max_dprimary = dprimary;
  if (dprimary < min_dprimary) min_dprimary = dprimary;
  if (clip) ++clipped;
}

float TestStats::rms() const { return samples ? sqrtf(float(sumsq / samples)) : 0.f; }

void fft(float* re, float* im, size_t n) {
  // Bit-reversal permutation
  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      std::swap(re[i], re[j]);
      std::swap(im[i], im[j]);
    }
  }
  // Danielson-Lanczos butterflies
  for (size_t len = 2; len <= n; len <<= 1) {
    const float ang = -2.f * float(M_PI) / float(len);
    const float wr = cosf(ang), wi = sinf(ang);
    for (size_t i = 0; i < n; i += len) {
      float cr = 1.f, ci = 0.f;
      for (size_t k = 0; k < len / 2; ++k) {
        const size_t a = i + k, b = i + k + len / 2;
        const float tr = re[b] * cr - im[b] * ci;
        const float ti = re[b] * ci + im[b] * cr;
        re[b] = re[a] - tr;
        im[b] = im[a] - ti;
        re[a] += tr;
        im[a] += ti;
        const float ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = ncr;
      }
    }
  }
}

SpectrumPeak dominantFrequency(const float* x, size_t n, float fs, float fmin, float fmax,
                               float* re, float* im) {
  SpectrumPeak r;
  if (n < 16 || fs <= 0) return r;
  r.df_hz = fs / float(n);
  fmax = std::min(fmax, 0.5f * fs);  // never search beyond Nyquist
  if (fmax <= fmin) return r;

  // Remove the mean, apply a Hann window (reduces leakage from the finite record).
  double mean = 0;
  for (size_t i = 0; i < n; ++i) mean += x[i];
  mean /= double(n);
  float wsum = 0;
  for (size_t i = 0; i < n; ++i) {
    const float w = 0.5f * (1.f - cosf(2.f * float(M_PI) * float(i) / float(n - 1)));
    re[i] = (x[i] - float(mean)) * w;
    im[i] = 0;
    wsum += w;
  }
  fft(re, im, n);

  // Single-sided amplitude: |X_k| * 2 / sum(w) gives the sine amplitude for a
  // bin-centred tone (coherent-gain correction for the Hann window).
  const size_t half = n / 2;
  for (size_t k = 0; k <= half; ++k) re[k] = 2.f * sqrtf(re[k] * re[k] + im[k] * im[k]) / wsum;

  size_t k0 = size_t(ceilf(fmin / r.df_hz));
  size_t k1 = size_t(floorf(fmax / r.df_hz));
  if (k0 < 1) k0 = 1;
  if (k1 > half - 1) k1 = half - 1;
  if (k1 <= k0 + 2) return r;

  size_t kmax = k0;
  for (size_t k = k0; k <= k1; ++k) {
    im[k - k0] = re[k];  // copy band for the median estimate
    if (re[k] > re[kmax]) kmax = k;
  }
  const size_t nb = k1 - k0 + 1;
  std::nth_element(im, im + nb / 2, im + nb);
  r.noise_floor = im[nb / 2];

  // Parabolic interpolation on the three bins around the maximum.
  float delta = 0;
  if (kmax > k0 && kmax < k1) {
    const float a = re[kmax - 1], b = re[kmax], c = re[kmax + 1];
    const float den = a - 2.f * b + c;
    if (fabsf(den) > 1e-12f) delta = 0.5f * (a - c) / den;
  }
  r.freq_hz = (float(kmax) + delta) * r.df_hz;
  r.amp = re[kmax];
  r.valid = r.amp >= 4.f * r.noise_floor && r.amp > 0;  // >= 12 dB above band median
  return r;
}

}  // namespace dsp
