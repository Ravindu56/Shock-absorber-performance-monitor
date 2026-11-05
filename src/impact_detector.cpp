#include "impact_detector.h"

void ImpactDetector::configure(const ImpactParams& p, float fs_hz) {
  p_ = p;
  if (p_.persist_samples < 1) p_.persist_samples = 1;
  if (p_.window_samples < 1) p_.window_samples = 1;
  // At least one sample below the release level ends an event (0 would end it
  // on the first ACTIVE sample, since below_ >= 0 always holds).
  if (p_.release_hold_samples < 1) p_.release_hold_samples = 1;
  if (p_.hysteresis < 0) p_.hysteresis = 0;
  if (p_.hysteresis > p_.threshold) p_.hysteresis = p_.threshold;
  fs_ = fs_hz > 0 ? fs_hz : 1000.f;
  // An event in progress keeps going under the new parameters.
}

void ImpactDetector::reset() {
  state_ = State::IDLE;
  above_ = below_ = 0;
  count_ = 0;
  have_prev_ = false;
}

bool ImpactDetector::update(uint32_t idx, int64_t t_us, float dmag, const float d[3],
                            ImpactEvent& out) {
  switch (state_) {
    case State::REFRACTORY:
      if (int32_t(idx - refractory_until_) < 0) return false;
      state_ = State::IDLE;
      above_ = 0;
      [[fallthrough]];

    case State::IDLE:
      if (dmag > p_.threshold) {
        if (above_ == 0) {
          cur_ = ImpactEvent();
          cur_.start_idx = idx;
          cur_.t_start_us = t_us;
        }
        if (dmag > cur_.peak_dyn) {
          cur_.peak_dyn = dmag;
          cur_.peak_idx = idx;
          for (int i = 0; i < 3; ++i) cur_.peak_vec[i] = d[i];
        }
        if (++above_ >= p_.persist_samples) {
          state_ = State::ACTIVE;
          below_ = 0;
        }
      } else {
        above_ = 0;
      }
      return false;

    case State::ACTIVE: {
      if (dmag > cur_.peak_dyn) {
        cur_.peak_dyn = dmag;
        cur_.peak_idx = idx;
        for (int i = 0; i < 3; ++i) cur_.peak_vec[i] = d[i];
      }
      below_ = (dmag < p_.threshold - p_.hysteresis) ? below_ + 1 : 0;
      if (below_ >= p_.release_hold_samples) {
        finish(idx - below_ + 1, false, out);
        return true;
      }
      if (idx - cur_.start_idx + 1 >= p_.window_samples) {
        finish(idx + 1, true, out);
        return true;
      }
      return false;
    }
  }
  return false;
}

void ImpactDetector::finish(uint32_t end_idx, bool truncated, ImpactEvent& out) {
  cur_.end_idx = end_idx;
  cur_.truncated = truncated;
  cur_.duration_ms = float(end_idx - cur_.start_idx) * 1000.f / fs_;
  cur_.interval_ms = have_prev_ ? float(cur_.start_idx - prev_start_) * 1000.f / fs_ : -1.f;
  cur_.id = ++count_;
  have_prev_ = true;
  prev_start_ = cur_.start_idx;
  refractory_until_ = end_idx + p_.refractory_samples;
  state_ = State::REFRACTORY;
  above_ = below_ = 0;
  out = cur_;
}
