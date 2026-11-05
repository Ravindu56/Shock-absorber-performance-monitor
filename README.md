# Shock Absorber Performance Monitor — ESP32 + MPU6500

An MPU6500 IMU mounted on top of a shock absorber prototype measures the **Z-axis** (stroke-axis)
acceleration and vibration transmitted through the damper. X and Y are sampled only for calibration;
all measurements, plots and recordings use Z (the *Monitored axis* setting, `AXIS`, if the sensor is
mounted differently). An ESP32 samples it from the hardware FIFO, removes gravity, detects impacts and streams
everything over USB. A Python dashboard shows live graphs and numbers, analyses each impact (compression,
rebound, ring-down frequency, damping ratio, settling time) and records tests to CSV so you can compare
prototypes.

**Bandwidth:** with the MPU6500 on 400 kHz I²C, frequencies are **validated only up to 200 Hz**
(default profile). The requested **500–2000 Hz band cannot be measured** with this sensor. See
[Bandwidth and frequency limits](#bandwidth-and-frequency-limits) for why and what hardware would
cover it. The firmware and dashboard check the sampling setup and never report a frequency outside
the validated band as a measurement.

---

## Contents

1. [Hardware](#hardware)
2. [Project layout](#project-layout)
3. [Quick start](#quick-start) · [Windows installation (exe)](#windows-installation-exe)
4. [Firmware](#firmware)
5. [Dashboard](#dashboard)
6. [Measurements and signal processing](#measurements-and-signal-processing)
7. [Bandwidth and frequency limits](#bandwidth-and-frequency-limits)
8. [Calibration](#calibration)
9. [Shock absorber test procedure](#shock-absorber-test-procedure)
10. [Recorded files (CSV)](#recorded-files-csv)
11. [Serial protocol](#serial-protocol)
12. [Error handling and troubleshooting](#error-handling-and-troubleshooting)
13. [Testing](#testing)

---

## Hardware

| MPU6500 pin | ESP32 (DOIT DevKit V1) |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SDA | GPIO 21 (D21) |
| SCL | GPIO 22 (D22) |
| AD0 | not connected (address 0x68; 0x69 is auto-detected if AD0 is high) |
| INT | not connected (the firmware polls the FIFO) |
| **Zero push button** | GPIO 4 (D4) → button → GND (internal pull-up, no resistor needed) |
| Status LED | on-board LED, GPIO 2 (lit while zeroing) |

* **Pull-ups:** most MPU6500 breakout boards have 4.7–10 kΩ pull-ups to 3.3 V on SDA/SCL. A bare chip
  needs external ones (2.2–4.7 kΩ for 400 kHz).
* **Mounting:** bolt or rigidly glue the board to the top mount. Tape and foam act as a mechanical
  low-pass filter and add their own resonance. Keep the I²C wires short (< 30 cm) and strain-relieved;
  vibration fatigues jumper wires.
* **Orientation:** align one sensor axis with the damper stroke, then select it as the *primary
  axis* (default Z).
* WHO_AM_I 0x70 (MPU6500) is expected. 0x71/0x73 (MPU9250/9255 dies, same accelerometer and FIFO
  registers) are accepted and reported.

## Project layout

```
platformio.ini            ESP32 build (Arduino framework)
run_dashboard.py          dashboard launcher (also the .exe entry point)
packaging/                Windows build: PyInstaller spec, build_windows.bat, Inno Setup installer, icon
.github/workflows/        GitHub Actions: builds the Windows exe + installer
src/
  config.h                all constants, sensor profiles, RuntimeConfig defaults
  mpu6500_driver.*        register-level MPU6500 access (I²C 400 kHz, FIFO)
  acquisition.*           sampling task: FIFO polling, timing, DSP, impacts, calibration capture
  signal_processing.*     baseline removal, RMS/P2P windows, test statistics, FFT
  impact_detector.*       threshold/hysteresis/persistence/refractory state machine
  serial_protocol.*       line protocol, command parser
  config_store.*          save/load configuration in flash (NVS)
  main.cpp                report task: commands, status/event/raw/FFT output
dashboard/
  app.py                  PyQtGraph GUI           (python -m dashboard)
  session.py              GUI-independent state, event analysis scheduling
  analysis.py             NumPy processing: baseline, spectrum, sampling check, decay analysis
  protocol.py             line parser (mirrors serial_protocol.cpp)
  links.py                serial port / simulator connections (background threads)
  simulator.py            software device for testing without hardware
  calibration.py          6-position calibration solver, stationary check
  recorder.py             CSV recording and summaries
  compare.py              compare recorded tests  (python -m dashboard.compare)
  tests/                  unit + end-to-end tests (python -m unittest)
```

## Quick start

```bash
# 1. Firmware (from the project folder)
pio run -t upload            # build + flash
# (optional) pio device monitor   -> raw protocol lines at 921600 baud

# 2. Dashboard
python3 -m pip install -r dashboard/requirements.txt
python3 -m dashboard                         # choose the port, press Connect
python3 -m dashboard --port /dev/cu.usbserial-0001
python3 -m dashboard --simulate              # no hardware: built-in simulator
```

Only one program can open the serial port at a time, so close the PlatformIO monitor before you connect
the dashboard.

## Windows installation (exe)

The dashboard can be packaged as a normal Windows program: `ShockMonitor.exe` plus an installer
with Start menu and desktop shortcuts. Python is **not** needed on the PC where it is installed.
A Windows `.exe` must be **built on Windows** (PyInstaller cannot cross-compile from macOS), using
one of these two routes:

**A. On the Windows PC (one-click script)**

1. Install **Python 3.12 (64-bit)** from python.org and tick *Add python.exe to PATH*.
2. Install **Inno Setup 6** (jrsoftware.org) to get the installer. Without it you still get the
   program folder.
3. Copy this project folder to the PC and double-click `packaging\build_windows.bat`.
   It creates a private build environment, installs the packages, runs the tests, and builds:
   * `dist\installer\ShockMonitor-Setup-1.1.0.exe`: **the installer to use**
   * `dist\ShockMonitor\ShockMonitor.exe`: portable version (run directly, no install)

**B. On GitHub (no Python on Windows needed)**

Push the project to a GitHub repository, open *Actions → Windows build → Run workflow*, and download
the `ShockMonitor-windows` artifact (installer + portable folder). The workflow
(`.github/workflows/windows-build.yml`) also starts the built exe with the simulator to check that it runs.

**Using it (plug and play)**

1. Run the installer, then start *Shock Absorber Monitor* from the Start menu or desktop.
2. Plug in the ESP32 (already flashed with the firmware). The app finds it by its USB chip
   (CP210x, CH340/CH9102 or FTDI) and **connects automatically**. If the cable is unplugged and
   replugged, it reconnects automatically.
3. Recordings are saved to `Documents\ShockMonitor\recordings`. If the program ever fails, the
   details are written to `Documents\ShockMonitor\crash.log`.

**USB driver:** the DOIT DevKit V1 uses a Silicon Labs **CP2102**. Windows 10/11 normally installs its
driver automatically on first plug-in. If no new COM port appears in Device Manager, install the
*CP210x Universal Windows Driver* from silabs.com (for CH340 boards: the WCH CH341SER driver).

The firmware is flashed once from the development computer (`pio run -t upload`). The Windows PC
only needs the dashboard.

## Firmware

### Architecture

```
 MPU6500 FIFO ──I²C 400 kHz──► acq task (core 1, prio 5, every 2 ms)
                                 │ read INT_STATUS + FIFO_COUNT + all complete 6-byte frames
                                 │ per sample: counts → m/s² → calibration → baseline → d
                                 │            window RMS/P2P, test stats, impact detector, FFT ring
                                 ├─► snapshot (spinlock)          ─┐
                                 ├─► raw sample ring (lock-free)   ├─► report task (core 0, prio 2)
                                 ├─► event / CAL queues           ─┘     Serial 921600: S, TS, E, F, R, CAL, CFG
                                 ◄── command queue ◄──────────────────── Serial commands
```

* **FIFO acquisition.** The sensor writes each accelerometer sample into its 512-byte FIFO on its own
  clock. The ESP32 reads all complete frames every 2 ms (2 samples at 1 kHz). The FIFO holds about 85 ms
  of data, so Serial output, flash writes and FFTs never cause lost samples. The INT pin isn't wired.
  A data-ready interrupt would mean one I²C transaction per sample and a missed sample on every jitter,
  while FIFO bursts are cheaper and robust.
* **Overflow handling.** FIFO_MODE = 1 stops writing when full, which keeps frame alignment. If an
  overflow is seen, the FIFO is reset, a gap is recorded and the sample index advances by the number
  of missed samples, so `idx / fs` still tracks time.
* **Timing.** With the gyro off, the MPU6500 runs from its internal oscillator, which can be a few percent
  off. The firmware measures the true sample rate against the ESP32 crystal every second (`fs_meas`).
  All frequencies (device FFT and dashboard) use the measured rate. A deviation above 3% raises
  `RATE_DEV` and invalidates frequency results.
* **Gyro** is in standby (not needed; reduces noise coupling and power).

### Build settings

`platformio.ini`: `-O2 -Wall -Wextra`, 240 MHz, 921600 baud monitor/upload. Builds without warnings.
Uses about 28% RAM and 25% flash.

### Configuration

Compile-time constants (pins, I²C clock, buffer sizes, task layout, error limits) are in `src/config.h`.
Runtime parameters can be changed from the dashboard's **Device settings** panel or with
`SET <key> <value>`:

| Key | Default | Range | Meaning |
|---|---|---|---|
| `PROFILE` | STD_1K_218 | see [profiles](#sensor-profiles) | ODR and DLPF bandwidth (re-initialises the sensor) |
| `RANGE` | 16 | 2, 4, 8, 16 | full scale ±g (re-initialises the sensor) |
| `AXIS` | Z | X, Y, Z | primary (stroke) axis for P2P, FFT and decay |
| `THRESHOLD` | 9.81 | 0.05–1000 m/s² | impact trigger level on \|d<sub>z</sub>\| |
| `HYSTERESIS` | 2.0 | < threshold | event ends below threshold − hysteresis |
| `PERSIST` | 2 | 1–100 samples | consecutive samples above threshold to trigger |
| `WINDOW_MS` | 300 | 1–10000 | maximum event length (longer → `truncated`) |
| `RELEASE_MS` | 20 | 0–1000 | time below release level that ends an event (min. 1 sample) |
| `REFRACTORY_MS` | 200 | 0–60000 | dead time after an event |
| `TAU` | 0.5 | 0.05–30 s | baseline (gravity) time constant; high-pass corner 1/(2πτ) = 0.32 Hz |
| `RMS_MS` | 200 | 10–10000 | RMS/P2P window |
| `REPORT_HZ` | 20 | 1–100 | status lines per second |
| `FFT_N` | 1024 | 256–2048 (2ⁿ) | device FFT length; resolution fs/N |
| `FFT_MS` | 1000 | 200–60000 | device FFT interval |
| `FMIN` | 2.0 | 0–500 Hz | lowest frequency considered for the dominant peak |
| `STREAM` | 1 | 0/1 | stream full-rate raw samples (needed for graphs, decay, recording) |
| `OFS_X/Y/Z` | 0 | ±20 m/s² | calibration offset |
| `TARE_X/Y/Z` | 0 | ±200 m/s² | zero offset (set by the button / `TARE`; 0 = include gravity) |
| `SCL_X/Y/Z` | 1 | 0.8–1.2 | calibration scale |

`SAVE` stores the active configuration (including calibration) in flash, where it is loaded at boot.
`DEFAULTS` erases it and restores the compiled defaults.

## Dashboard

```
python3 -m dashboard [--port PORT] [--baud 921600] [--simulate] [--record-dir DIR] [--buffer-s 60]
```

| Area | Content |
|---|---|
| **Toolbar** | port, connect, Start/Stop acquisition, Reset stats, units (m/s² or g), freeze plots, test label, Record, Export snapshot |
| **Live** tab | Z acceleration (calibrated, with gravity); Z vibration with ± threshold lines and impact markers |
| **Spectrum** tab | Welch amplitude spectrum of the Z vibration; non-validated band shaded grey; DLPF corner; dominant peak (host and device) |
| **Impact analysis** tab | table of all impacts with compression, rebound, ratio, f<sub>d</sub>, ζ, settling, R², notes; plot of the selected event with ring-down lobes and fitted envelope |
| **Calibration** tab | stationary check; 6-position calibration wizard |
| **Log** tab | device messages, acknowledgements, errors; raw command entry |
| **Side panel** | Z acceleration (m/s² and g), Z window vibration (RMS, P2P, peak, dominant frequency), test totals, **sampling and bandwidth verification**, device flags, device settings, analysis settings |

The dashboard recomputes *a* and *d* from the raw sample stream with the same filter as the firmware
(the tests check they match), so graphs, spectra and recordings use every sample. The numbers in the
side panel's "device" groups come straight from the firmware, which processes every sample even if
the stream drops some.

**Analysis settings** (dashboard only): *compression sign* (which direction of the primary axis is
compression for your mounting), *decay window* (time after the peak analysed, default 1000 ms),
*settling band* (default 10% of peak) and *plot window*.

## Measurements and signal processing

| Quantity | Definition |
|---|---|
| a = (a<sub>x</sub>, a<sub>y</sub>, a<sub>z</sub>) | calibrated acceleration incl. gravity, m/s² (÷ 9.80665 for g) |
| baseline b | per-axis first-order low-pass of a, b += α(a − b), α = 1 − e<sup>−1/(fs·τ)</sup> |
| dynamic acceleration d | d = a − b (gravity and static tilt removed) |
| Z acceleration | a<sub>z</sub> in m/s² and g: ≈ 0 after zeroing (button), otherwise includes gravity (≈ +1 g when Z points up) |
| Z vibration | d<sub>z</sub> = a<sub>z</sub> − b<sub>z</sub> (gravity removed) |
| RMS | √(mean(d<sub>z</sub>²)) over the RMS window, and since the last reset |
| Peak | max \|d<sub>z</sub>\| (per status interval, per event, per test) |
| Peak-to-peak | max(d<sub>z</sub>) − min(d<sub>z</sub>) |
| Dominant frequency | largest spectral peak in [FMIN, validated max], ≥ 12 dB above the band median; parabolic interpolation between bins |

*d* is not a true gravity-free linear acceleration. If the sensor tilts faster than τ, the change in
gravity projection appears in *d*. For a top mount that barely rotates this is negligible. Check it
with the stationary calibration report and by watching *d* while slowly tilting the assembly.

### Impact detection

```
IDLE ──|d| > threshold for PERSIST samples──► ACTIVE (track peak)
ACTIVE ──|d| < threshold − hysteresis for RELEASE_MS──► event ends
ACTIVE ──lasted WINDOW_MS──► event ends (truncated)
event end ──► REFRACTORY for REFRACTORY_MS ──► IDLE
```

The detector works on \|d<sub>z</sub>\|, so sideways (X/Y) motion never triggers an event.
Persistence rejects single-sample spikes. Hysteresis and release hold stop a ringing response from
splitting into many events. The refractory time prevents double counts. Each event reports start/peak/end
sample, peak \|d\| and the d vector at the peak, duration and the interval to the previous event.

### Compression, rebound and decay analysis (dashboard)

For each event the dashboard takes the primary-axis signal from 100 ms before the event to 1 s after
the peak. It references the signal to the mean acceleration just before the impact. This is the
static level; the live EMA baseline would absorb part of the ring-down's own mean.

1. **Half-cycles / lobes:** the signal is split at zero crossings (with a 5σ-noise deadband). The largest
   \|x\| in each half-cycle is a lobe.
2. **Compression / rebound:** compression = largest lobe with the compression sign; rebound = the next
   opposite-sign lobe; **rebound ratio** = \|rebound\| / \|compression\| (lower = more damping on rebound).
   These use the unfiltered signal.
3. **Ring-down:** alternating, non-growing lobes from the largest one, above max(5σ, 3% of peak). A
   coarse damped frequency sets a zero-phase low-pass at 2.5·f<sub>d</sub> that removes impact transients
   and higher modes. The lobes are then re-extracted from the filtered signal (shown in the event plot):
   * damped frequency f<sub>d</sub> = 1 / (2 × mean zero-crossing spacing)
   * decay rate σ from a least-squares fit of ln(A<sub>k</sub>) against t<sub>k</sub>
   * damping ratio **ζ = σ / √(σ² + (2πf<sub>d</sub>)²)**, logarithmic decrement δ = σ / f<sub>d</sub>
   * fit R² (below 0.9 means the decay is not exponential, e.g. friction or nonlinear damping, and ζ is
     only indicative)
4. **Settling time:** from the main lobe until \|x\| stays inside the settling band (10% of peak).
5. With only one overshoot, ζ comes from the overshoot ratio, ln(A₁/A₂) = πζ/√(1−ζ²), and is
   flagged as approximate. With no overshoot above noise, the result is *no oscillation* (ζ ≳ 0.6 or
   weak excitation).

Accuracy is checked by the tests on synthetic ring-downs: ζ within 5% and f<sub>d</sub> within 2% for
ζ = 0.05–0.3. On the simulator (impact transient + 47 Hz vibration + noise), the mean over a test is
within 3% of ζ = 0.25 and 1% of f<sub>d</sub>. Single events scatter by about ±0.02 in ζ, so compare
prototypes by the mean ± std over several impacts (as `summary.csv` and `compare.py` do).
Ring-down frequencies above the validated band are flagged as not validated.

## Bandwidth and frequency limits

A frequency is only reportable when all of the following hold:

1. **The sample rate is verified:** `fs_meas` (ESP32 crystal timebase) is within 3% of nominal, no
   excessive duplicate samples, and < 1% of samples missing from the stream.
2. **It lies below the sensor's analog bandwidth with margin:** the on-chip DLPF is a gentle roll-off,
   not a brick wall. The response is already −3 dB at the corner frequency.
3. **It cannot be an alias:** content above Nyquist (fs/2) folds back into the band. Nyquist must be
   well above the DLPF corner so out-of-band vibration is attenuated before it aliases.

The dashboard shows each check in **Sampling and bandwidth verification**. Spectra and ring-down
frequencies outside the validated band are shaded and labelled *not validated*.

### Sensor profiles

| Profile | ODR | Accel DLPF (−3 dB) | Nyquist | Validated band | Use |
|---|---|---|---|---|---|
| `STD_1K_218` (default) | 1 kHz | 218 Hz | 500 Hz | **≤ 200 Hz** | general shock and vibration |
| `LN_1K_99` | 1 kHz | 99 Hz | 500 Hz | ≤ 90 Hz | lower noise, strong anti-aliasing |
| `LN_500_45` | 500 Hz | 45 Hz | 250 Hz | ≤ 40 Hz | slow body/suspension motion |
| `EXP_1K_420` | 1 kHz | 420 Hz | 500 Hz | **none** (`ALIAS_RISK`) | experimental only; 420 Hz DLPF with 500 Hz Nyquist aliases |

Bandwidths are from the MPU-6500 register map (A_DLPF_CFG). The MPU-9250 register map lists different
values for the same settings (e.g. 184 Hz instead of 218 Hz), which is one more reason the validated band
keeps a margin below the corner. To check your unit, excite the mount at a known frequency (signal
generator + small shaker or speaker, 10–300 Hz) and compare the reported peak and amplitude.

### Why 500–2000 Hz is not measurable here

* **Sensor bandwidth.** The MPU6500 accelerometer's widest setting (DLPF bypassed, ACCEL_FCHOICE_B = 1)
  has about 1.1 kHz bandwidth at 4 kHz output rate. Nothing above about 1.1 kHz reaches the output
  with usable amplitude, so **1.1–2 kHz is outside the sensor's capability on any bus**.
* **I²C throughput.** To capture that 4 kHz mode through the FIFO, the internal sample rate must be 8 kHz,
  which writes 8000 × 6 B = 48 kB/s into the FIFO. 400 kHz I²C moves at most about 44 kB/s (9 bit-times
  per byte) before any addressing overhead. Polling the data registers at 4 kHz instead takes about
  85% of the bus with no timing margin.
* **Aliasing.** Even in the 4 kHz mode, the 1.1 kHz single-stage roll-off provides little attenuation
  at 2–4 kHz, so higher-frequency vibration folds into 500–1100 Hz.

At best, an SPI-connected MPU6500 in the 4 kHz mode could give *indicative* (not validated) content up
to about 1 kHz. **For a properly validated 500–2000 Hz measurement use a dedicated vibration
accelerometer**, for example:

* ST **IIS3DWB** (3-axis, digital SPI, ~6 kHz bandwidth, 26.7 kHz ODR)
* ADI **ADXL1002** (single-axis analog, ~11 kHz bandwidth) with an anti-alias filter and an ADC sampling
  at ≥ 5 kHz
* an IEPE/piezo accelerometer with signal conditioning

These need firmware changes (SPI driver, higher-rate streaming in binary instead of base64). The
current architecture (FIFO burst reads, SPSC rings, block streaming) carries over.

## Zero button (tare)

Press and release the button on GPIO 4 (or *Zero Z* in the dashboard, or send `TARE`) while the
sensor is **still and in its test position**. The firmware waits 0.5 s for the press vibration to die
out, then averages Z for 1 s. It subtracts that value from then on, so **Z reads 0 m/s² at rest:
gravity and static offset are removed** from the displayed and recorded Z acceleration.

* LED: lit while zeroing → **3 slow blinks** = done, **10 fast blinks** = refused (the sensor moved,
  std > 0.3 m/s², or acquisition is stopped). The dashboard log shows the reason.
* The zero offset is **saved to flash** automatically, so it survives power-off. *Clear zero* (or
  `SET TARE_Z 0` + `SAVE`) shows gravity again.
* The zero offset is separate from the [calibration](#calibration) (offset/scale), so the stationary
  |g| check keeps working. Impact detection pauses while zeroing and the baseline restarts, so
  pressing the button never counts as an impact.
* Vibration, impacts and decay analysis use the gravity-removed signal either way, so zeroing changes
  only the Z *acceleration* reading.
* Re-zero after you remount or tilt the sensor.

## Calibration

Datasheet tolerances for the MPU6500 are a few percent of sensitivity and up to several hundred mg of
zero-g offset. Offsets do not affect *d* (the baseline removes them), but scale errors directly
change every amplitude.

**Stationary check** (Calibration tab → *Run stationary check*, or `CAL 3000`): keep the sensor still
for 3 s. Reports \|g\| (should be 9.807 m/s² ± 2%), noise per axis (mg RMS, about 5–6 mg expected at
218 Hz bandwidth) and roll/pitch tilt. Run it before every test session.

**6-position calibration:**

1. Rest the sensor (or the whole assembly) so +X points straight up, keep it still, press *Capture +X
   axis up*. Repeat for −X, +Y, −Y, +Z, −Z. A level surface and a square block help.
2. *Compute*: offset<sub>i</sub> = (up + down)/2, scale<sub>i</sub> = 2g / (up − down). The dashboard
   warns about movement, wrong orientation or inconsistent captures, and shows the \|g\| residual for
   each position.
3. *Apply to device* (sends `SET OFS_*/SCL_*`), then *Save to flash*.

The firmware applies a<sub>cal</sub> = (a − offset) × scale before all processing. CAL results are
always uncalibrated, so you can repeat the procedure at any time without clearing the calibration
first.

## Shock absorber test procedure

1. **Mount and orient** the sensor rigidly on the top mount. Pick the stroke axis as *Primary axis*.
2. **Stationary check** (above): confirm \|g\|, noise and tilt.
3. **Find the compression sign:** press the damper slowly by hand and watch the primary-axis plot. If
   compression shows as negative, set *Compression sign* to −.
4. **Choose the range:** start at ±16 g. If test peaks stay below about 3 g, ±4 g gives finer
   resolution. Any `CLIP` flag or clipped samples mean the range is too small.
5. **Set the threshold** to about 5–10× the quiet RMS \|d\| shown in *Vibration*, and above normal
   running vibration. Raise PERSIST if electrical spikes trigger events.
6. **Check sampling:** the side panel must show *Spectral data: VALID* before you use any frequency.
7. For each prototype: enter a **test label**, press **Record** (resets device statistics), run the
   same input sequence (drop height, bump, rig stroke, ≥ 10 impacts), then stop.
8. **Compare** the recordings:

   ```bash
   python3 -m dashboard.compare recordings/* -o comparison.csv
   ```

   Lower transmitted peak, lower rebound ratio and shorter settling usually mean better damping and
   isolation. ζ and f<sub>d</sub> describe the damper/spring/mass system as mounted, so compare
   prototypes on the same rig, mass and input. Low R² values or *irregular lobe spacing* notes indicate
   nonlinear or multi-mode behaviour, so don't compare those ζ values directly.

## Recorded files (CSV)

`recordings/<YYYYmmdd_HHMMSS>_<label>/`:

| File | Content |
|---|---|
| `samples.csv` | `idx, t_s, az_ms2, az_g, dz_ms2, dz_g, raw_z`: every streamed Z sample. t = idx / fs<sub>meas</sub>. Gaps appear as jumps in `idx`. |
| `status.csv` | every device status line (S fields + host time) |
| `events.csv` | per impact: device fields + `compression_peak_ms2, rebound_peak_ms2, rebound_ratio, comp_to_rebound_ms, lobes, damped_freq_hz, freq_valid, decay_rate_per_s, damping_ratio, log_decrement, fit_r2, settling_ms, method, notes` |
| `spectra.csv` | device FFT results (dominant frequency, amplitude, noise floor, valid) |
| `summary.csv` | `key,value`: device identity, full configuration, sampling verification, test statistics, mean/std/min/max of the event metrics. `compare.py` reads this file. |

*Export snapshot* writes the same structure from the in-memory history (last 60 s by default) without
needing a recording in progress.

## Serial protocol

921600 baud, 8N1, one ASCII line per message, fields comma-separated, accelerations in m/s². The `H`
lines sent after `HELLO` name the columns of every line type, so the format describes itself.

| Line | Rate | Fields |
|---|---|---|
| `HELLO` | boot / on request | protocol version, firmware version, I²C address, WHO_AM_I, init status |
| `H,<type>,...` | after HELLO | column names for S, TS, E, F, R, CAL |
| `CFG,k=v,...` | on change | complete active configuration incl. profile bandwidths and calibration |
| `S` | `REPORT_HZ` | idx, t_ms, ax..az, \|a\|, dx..dz, \|d\|, rms_win, p2p_win, peak_int, test_max_dyn, test_rms, test_p2p, impacts, impact_active, fs_meas, flags, fifo_ovf, i2c_err, stream_drop, gaps, dup_ratio |
| `TS` | 1 Hz | whole-test min/max per axis, max \|d\|, primary min/max, RMS, P2P, impacts, clipped |
| `E` | per impact | id, start/peak/end idx, t_start_ms, peak \|d\|, d vector at peak, duration, interval, truncated |
| `F` | `FFT_MS` | idx, N, fs, df, fmin, fmax, dominant Hz, amplitude, noise floor, valid |
| `R` | ~20/s | gen, first_idx, n, base64(int16 LE x,y,z × n): raw counts, up to 50 samples |
| `CAL` | after `CAL` | samples, mean x/y/z (uncalibrated), std x/y/z, \|mean\|, roll, pitch, clipped, fs_meas |
| `ACK,<cmd>,OK\|ERR,<msg>` | per command | |
| `ERR,<code>,<msg>` | on error | e.g. `SENSOR` init failure with wiring hint |
| `# text` | | log line |

**Commands:** `PING`, `HELLO`, `GETCFG`, `START`, `STOP`, `RESET`, `CAL [ms]`, `SET <key> <value>`,
`SAVE`, `DEFAULTS`, `TARE` (zero the monitored axis, same as the button).

**Status flags** (`S.flags` bits): 0 SENSOR_OK, 1 RUNNING, 2 FIFO_OVF, 3 I2C_ERR, 4 CLIP, 5 RATE_DEV,
6 STREAM_DROP, 7 ALIAS_RISK, 8 RECOVERING, 9 DUPLICATES, 10 GAP. Bits 2, 3, 4, 6 and 10 are latched
between status lines.

`dashboard/tests/test_firmware_sync.py` parses `src/serial_protocol.cpp` and fails if its column
lists or field counts drift from the dashboard parser.

## Error handling and troubleshooting

| Mechanism | Behaviour |
|---|---|
| Sensor not found / wrong WHO_AM_I / register verify failure | `ERR,SENSOR,...` with wiring hint; retried every second |
| I²C transaction errors | counted (`I2C_ERR`); after 10 consecutive failures the bus and sensor are re-initialised (`RECOVERING`) |
| FIFO overflow | FIFO reset, gap marked, index advanced by missed samples (`FIFO_OVF`, `GAP`) |
| Raw stream buffer full | samples not transmitted are counted (`STREAM_DROP`); device statistics remain complete |
| Sensor not updating | > 20% identical consecutive samples → `DUPLICATES` |
| Sample rate off | > 3% from nominal → `RATE_DEV`, frequencies invalidated |
| Full-scale hits | `CLIP` flag, clipped-sample counter |
| Invalid commands / values | `ACK,<cmd>,ERR,<reason>`, configuration unchanged |
| Dashboard | port errors shown in dialogs/log; parse errors counted; automatic `HELLO` re-request; device reset detected (index goes backwards) and buffers cleared |

| Symptom | Check |
|---|---|
| `ERR,SENSOR,sensor init failed (NOT_FOUND)` | wiring SDA=21/SCL=22, 3.3 V, GND, pull-ups |
| Dashboard says NO DATA | correct port? PlatformIO monitor closed? baud 921600? Press EN on the board. |
| Garbage characters at connect | ESP32 boot ROM messages at 115200 baud; harmless |
| `RATE_DEV` | sensor clock out of tolerance or I²C errors; check wiring and power |
| Frequent false impacts | raise THRESHOLD or PERSIST; check that the board is rigidly mounted |
| `CLIP` | increase RANGE |
| ζ marked approximate / low R² | nonlinear or friction damping, several modes, or a second impact inside the decay window |

## Testing

```bash
pio run                                            # firmware compiles with -Wall -Wextra, no warnings
python3 -m unittest discover -s dashboard/tests -t .   # 22 tests
python3 -m dashboard --simulate                    # full GUI without hardware
```

The tests cover decay-analysis accuracy on synthetic ring-downs, spectrum amplitude and frequency, the
sampling checks, host/firmware baseline-filter equivalence, buffer wrap-around, the protocol parser and
its sync with the firmware source, 6-position calibration recovering known sensor errors, and an
end-to-end simulated test (simulator → parser → analysis → CSV recording → compare).

The simulator models a 0.8%-fast sensor clock, offset/scale errors, noise, a 47 Hz vibration tone and
damped bumps (14 Hz, ζ = 0.25) with an impact transient, and uses a port of the firmware's
processing. It validates the dashboard, **not** the firmware's behaviour on real hardware. Before
trusting results, check the hardware with the stationary check, the 6-position calibration and a
known-frequency excitation.
