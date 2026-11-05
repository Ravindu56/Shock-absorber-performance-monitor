// config_store.h — persist RuntimeConfig (thresholds, profile, calibration...)
// in the ESP32 NVS flash partition so a calibrated unit keeps its settings
// across power cycles.
#pragma once

#include "config.h"

namespace store {

// Load the saved configuration into `out`. Returns false (and leaves `out`
// untouched) if nothing is saved or the stored blob is from another layout.
bool load(RuntimeConfig& out);

bool save(const RuntimeConfig& c);

// Erase the saved configuration (next boot uses compiled-in defaults).
bool erase();

// Clamp a loaded/received configuration to the ranges the firmware supports.
void sanitize(RuntimeConfig& c);

}  // namespace store
