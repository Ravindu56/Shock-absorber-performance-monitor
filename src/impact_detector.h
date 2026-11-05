// impact_detector.h — threshold-based impact/event detection on |d_primary|
// (absolute primary-axis dynamic acceleration, Z by default).
//
// State machine (all timing in samples, converted from ms at configure()):
//
//   IDLE --(|d| > threshold for `persist` consecutive samples)--> ACTIVE
//   ACTIVE: track peak |d|; the event ends when |d| stays below
//           (threshold - hysteresis) for `release_hold`, or when the event
//           has lasted `event_window` (flagged as truncated)
//   ACTIVE --(event ends)--> REFRACTORY --(refractory elapsed)--> IDLE
//
// The persistence count rejects single-sample noise spikes; hysteresis and
// release hold stop an oscillating ring-down from producing many short events;
// the refractory interval prevents duplicate triggers from one disturbance.
#pragma once

#include <stdint.h>

struct ImpactParams {
  float threshold = 9.81f;          // m/s^2
  float hysteresis = 2.0f;          // m/s^2
  uint32_t persist_samples = 2;
  uint32_t window_samples = 300;
  uint32_t release_hold_samples = 20;
  uint32_t refractory_samples = 200;
};

struct ImpactEvent {
  uint32_t id = 0;            // 1-based count since last reset
  uint32_t start_idx = 0;     // first sample above threshold
  uint32_t peak_idx = 0;
  uint32_t end_idx = 0;       // first sample of the final below-release run (or window end)
  float peak_dyn = 0;         // peak |d| [m/s^2]
  float peak_vec[3] = {0, 0, 0};  // d at the peak sample [m/s^2]
  float duration_ms = 0;
  float interval_ms = -1;     // start-to-start time since previous event, -1 for the first
  bool truncated = false;     // event hit the event window limit
  int64_t t_start_us = 0;     // ESP32 time estimate of start sample
};

class ImpactDetector {
 public:
  void configure(const ImpactParams& p, float fs_hz);
  void reset();  // clear state and event count
  // Feed one sample. Returns true when an event has completed (written to `out`).
  bool update(uint32_t idx, int64_t t_us, float dmag, const float d[3], ImpactEvent& out);

  bool active() const { return state_ == State::ACTIVE; }
  uint32_t count() const { return count_; }

 private:
  enum class State : uint8_t { IDLE, ACTIVE, REFRACTORY };
  void finish(uint32_t end_idx, bool truncated, ImpactEvent& out);

  ImpactParams p_;
  float fs_ = 1000.f;
  State state_ = State::IDLE;
  uint32_t above_ = 0, below_ = 0;
  uint32_t count_ = 0;
  ImpactEvent cur_;
  bool have_prev_ = false;
  uint32_t prev_start_ = 0;
  uint32_t refractory_until_ = 0;
};
