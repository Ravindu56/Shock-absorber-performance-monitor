#include "config_store.h"

#include <Preferences.h>
#include <math.h>

namespace store {

namespace {

// Stored blob: magic + size guard against loading a struct of a different layout.
struct Blob {
  uint32_t magic;
  uint32_t size;
  RuntimeConfig cfg;
};

template <typename T>
void clampv(T& v, T lo, T hi) {
  if (!(v >= lo)) v = lo;  // also catches NaN for floats
  if (v > hi) v = hi;
}

}  // namespace

void sanitize(RuntimeConfig& c) {
  if (c.profile >= PROFILE_COUNT) c.profile = PROFILE_STD_1K_218;
  c.range &= 0x03;
  c.primary_axis %= 3;
  clampv(c.baseline_tau_s, 0.05f, 30.f);
  clampv(c.rms_window_ms, uint32_t(10), uint32_t(10000));
  clampv(c.impact_threshold, 0.05f, 1000.f);
  clampv(c.impact_hysteresis, 0.f, c.impact_threshold * 0.99f);
  clampv(c.impact_persist_samples, uint32_t(1), uint32_t(100));
  clampv(c.event_window_ms, uint32_t(1), uint32_t(10000));
  clampv(c.release_hold_ms, uint32_t(0), uint32_t(1000));
  clampv(c.refractory_ms, uint32_t(0), uint32_t(60000));
  clampv(c.report_hz, uint32_t(1), uint32_t(100));
  if (c.fft_n < 256 || c.fft_n > cfg::FFT_MAX_N || (c.fft_n & (c.fft_n - 1)) != 0) c.fft_n = 1024;
  clampv(c.fft_interval_ms, uint32_t(200), uint32_t(60000));
  clampv(c.fft_fmin_hz, 0.f, 500.f);
  for (int i = 0; i < 3; ++i) {
    clampv(c.offset[i], -20.f, 20.f);
    clampv(c.scale[i], 0.8f, 1.2f);
    clampv(c.tare[i], -200.f, 200.f);
  }
}

bool load(RuntimeConfig& out) {
  Preferences p;
  if (!p.begin(cfg::NVS_NAMESPACE, true)) return false;  // read-only; fails if namespace absent
  Blob b;
  const size_t n = p.getBytes(cfg::NVS_KEY, &b, sizeof(b));
  p.end();
  if (n != sizeof(b) || b.magic != cfg::NVS_MAGIC || b.size != sizeof(RuntimeConfig)) return false;
  sanitize(b.cfg);
  out = b.cfg;
  return true;
}

bool save(const RuntimeConfig& c) {
  Preferences p;
  if (!p.begin(cfg::NVS_NAMESPACE, false)) return false;
  Blob b{cfg::NVS_MAGIC, sizeof(RuntimeConfig), c};
  const size_t n = p.putBytes(cfg::NVS_KEY, &b, sizeof(b));
  p.end();
  return n == sizeof(b);
}

bool erase() {
  Preferences p;
  if (!p.begin(cfg::NVS_NAMESPACE, false)) return false;
  const bool ok = p.clear();
  p.end();
  return ok;
}

}  // namespace store
