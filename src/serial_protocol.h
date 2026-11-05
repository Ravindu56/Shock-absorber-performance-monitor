// serial_protocol.h — line-oriented text protocol over USB Serial.
//
// Every line is "<TYPE>,<field>,<field>..." terminated by "\n". All
// accelerations are in m/s^2, times in ms, frequencies in Hz. The full
// specification is in README.md ("Serial protocol"); dashboard/protocol.py
// must stay in sync with the field lists defined in serial_protocol.cpp.
//
// Device -> host:
//   HELLO  protocol/firmware version, sensor address, WHO_AM_I, init status
//   CFG    active configuration as key=value pairs
//   H      column header for another line type (self-describing format)
//   S      status at report_hz: latest values, window stats, flags, counters
//   TS     whole-test statistics at 1 Hz (since last RESET)
//   E      completed impact event
//   F      on-device FFT result (dominant frequency)
//   R      raw sample block: gen, first index, count, base64(int16 LE x,y,z ...)
//   CAL    stationary calibration result
//   ACK    command acknowledgement
//   ERR    error report
//   #      free-text log line (ignored by parsers)
//
// Host -> device (one command per line, case-insensitive):
//   PING | HELLO | GETCFG | START | STOP | RESET | CAL [ms] | SET <key> <value>
//   SAVE (store configuration in flash) | DEFAULTS (erase it, restore defaults)
//   TARE (zero the monitored axis, same as the GPIO4 button; saved to flash)
#pragma once

#include <Arduino.h>

#include "acquisition.h"
#include "impact_detector.h"
#include "signal_processing.h"

namespace proto {

constexpr int PROTOCOL_VERSION = 1;
constexpr const char* FIRMWARE_VERSION = "1.0.0";

void sendHello();
void sendHeaders();
void sendConfig(const RuntimeConfig& c, uint32_t gen);
void sendStatus(const AcqSnapshot& s);
void sendTestStats(const AcqSnapshot& s);
void sendEvent(const ImpactEvent& e);
void sendSpectrum(uint32_t idx, size_t n, float fs, float fmin, float fmax,
                  const dsp::SpectrumPeak& p, bool valid);
void sendRaw(const RawSample* s, size_t n);
void sendCalibration(const CalResult& c);
void sendAck(const char* cmd, bool ok, const char* msg);
void sendError(const char* code, const char* msg);
void sendLog(const char* msg);

// Non-blocking: accumulates Serial input; returns true when a full line is in `line`.
bool readLine(char* line, size_t cap);

// Parsed host command.
enum class Command : uint8_t {
  NONE, PING, HELLO, GETCFG, START, STOP, RESET, CAL, SET, SAVE, DEFAULTS, TARE, INVALID
};

struct ParsedCommand {
  Command cmd = Command::NONE;
  uint32_t cal_ms = 2000;
  char name[16] = {0};   // command word, upper-case (for ACK)
  char error[64] = {0};  // reason when INVALID
};

// Parse one line. For SET the change is validated and applied to `desired`
// (which is left untouched on error).
ParsedCommand parseCommand(char* line, RuntimeConfig& desired);

}  // namespace proto
