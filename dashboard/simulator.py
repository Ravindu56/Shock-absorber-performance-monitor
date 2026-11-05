"""Software stand-in for the ESP32 firmware (``python -m dashboard --simulate``).

Produces the same protocol lines as the firmware (HELLO/H/CFG/S/TS/E/F/R/CAL/ACK)
from a synthetic shock absorber response, so the dashboard, recorder and
analysis can be exercised and tested without hardware. The signal model:

* gravity on +Z with a small tilt, plus per-axis offset/scale errors
  (so the calibration procedure has something to correct)
* white noise (~0.04 m/s^2 RMS, similar to an MPU6500 at 218 Hz bandwidth)
* a weak 47 Hz vibration tone
* random bumps: a damped oscillation on Z (14 Hz, zeta 0.25) with a short
  120 Hz transient and small X/Y coupling
* the sensor sample clock runs 0.8 % fast (internal oscillator tolerance)

The detection logic is a direct port of the firmware's ImpactDetector.
"""

from __future__ import annotations

import base64
import math
from typing import List, Optional

import numpy as np

from .protocol import DEFAULT_HEADERS, G

PROFILES = {  # name: (odr, bw, valid_max, alias_risk) — mirrors SENSOR_PROFILES in config.h
    "STD_1K_218": (1000.0, 218.1, 200.0, 0),
    "LN_1K_99": (1000.0, 99.0, 90.0, 0),
    "LN_500_45": (500.0, 44.8, 40.0, 0),
    "EXP_1K_420": (1000.0, 420.0, 0.0, 1),
}
PROFILE_NAMES = list(PROFILES)
RANGES = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}

DEFAULT_CFG = dict(profile="STD_1K_218", range_g=16, baseline_tau_s=0.5, rms_window_ms=200,
                   primary_axis="Z", threshold=9.81, hysteresis=2.0, persist=2, event_window_ms=300,
                   release_hold_ms=20, refractory_ms=200, report_hz=20, stream=1, fft_n=1024,
                   fft_interval_ms=1000, fft_fmin_hz=2.0, ofs=[0.0, 0.0, 0.0], scl=[1.0, 1.0, 1.0],
                   tare=[0.0, 0.0, 0.0])

SET_KEYS = {  # SET key -> (cfg key, type, lo, hi)
    "TAU": ("baseline_tau_s", float, 0.05, 30), "THRESHOLD": ("threshold", float, 0.05, 1000),
    "HYSTERESIS": ("hysteresis", float, 0, 1000), "FMIN": ("fft_fmin_hz", float, 0, 500),
    "RMS_MS": ("rms_window_ms", int, 10, 10000), "PERSIST": ("persist", int, 1, 100),
    "WINDOW_MS": ("event_window_ms", int, 1, 10000), "RELEASE_MS": ("release_hold_ms", int, 0, 1000),
    "REFRACTORY_MS": ("refractory_ms", int, 0, 60000), "REPORT_HZ": ("report_hz", int, 1, 100),
    "FFT_MS": ("fft_interval_ms", int, 200, 60000), "STREAM": ("stream", int, 0, 1),
}

# Orientation used by the simulator's "SIM_ORIENT" command (6-position calibration).
ORIENT = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0), "-Y": (0, -1, 0),
          "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


class DeviceSimulator:
    def __init__(self, seed: Optional[int] = None, bumps: bool = True) -> None:
        self.rng = np.random.default_rng(seed)
        self.cfg = {k: (list(v) if isinstance(v, list) else v) for k, v in DEFAULT_CFG.items()}
        self.gen = 1
        self.bumps = bumps
        self.running = True
        self.clock_error = 1.008          # sensor oscillator runs 0.8 % fast
        self.true_ofs = np.array([0.18, -0.12, 0.30])   # m/s^2 offset error
        self.true_scl = np.array([1.012, 0.991, 1.017])  # sensitivity error
        self.gravity_dir = np.array([0.02, -0.03, 1.0])
        self.gravity_dir /= np.linalg.norm(self.gravity_dir)
        self.out: List[str] = []
        self.idx = 0
        self.t_us = 0.0
        self._next_bump = 1.5
        self._bumps: List[tuple] = []     # (t0, amplitude)
        self._init_dsp()
        self._status_acc = 0.0
        self._test_acc = 0.0
        self._fft_acc = 0.0
        self._cal: Optional[dict] = None
        self._tare: Optional[dict] = None
        self.fft_ring = np.zeros(4096)
        self.fft_count = 0

    # -- helpers ---------------------------------------------------------
    @property
    def odr(self) -> float:
        return PROFILES[self.cfg["profile"]][0]

    @property
    def fs_true(self) -> float:
        return self.odr * self.clock_error

    def _emit(self, s: str) -> None:
        self.out.append(s)

    def _init_dsp(self) -> None:
        fs = self.odr
        self.alpha = 1 - math.exp(-1 / (fs * self.cfg["baseline_tau_s"]))
        self.base: Optional[np.ndarray] = None
        self.win_n = max(2, round(self.cfg["rms_window_ms"] * fs / 1000))
        self.win = dict(n=0, sumsq=0.0, peak=0.0, pmax=0.0, pmin=0.0)
        self.win_res = dict(rms=0.0, p2p=0.0)
        self.peak_int = 0.0
        if not hasattr(self, "test"):
            self.reset_stats()
        self.det_state = "IDLE"
        self.det_above = self.det_below = 0
        self.det_cur: dict = {}
        self.stream_buf: List[np.ndarray] = []
        self.stream_first = None

    def reset_stats(self) -> None:
        self.test = dict(samples=0, sumsq=0.0, max_dyn=0.0, max_raw=[0.0] * 3, min_raw=[0.0] * 3,
                         max_dp=0.0, min_dp=0.0, clipped=0)
        self.impacts = 0
        self.det_prev_start = None
        self.det_refr_until = 0
        self.det_state = "IDLE"
        self.det_above = self.det_below = 0

    # -- output lines ----------------------------------------------------
    def hello(self) -> None:
        self._emit("HELLO,1,1.0.0-sim,0x68,0x70,OK")
        for k, names in DEFAULT_HEADERS.items():
            self._emit(f"H,{k}," + ",".join(names))
        self._emit("H,R,gen,first_idx,n,b64_int16le_xyz")
        self.send_cfg()

    def send_cfg(self) -> None:
        c = self.cfg
        odr, bw, vmax, alias = PROFILES[c["profile"]]
        self._emit(
            f"CFG,gen={self.gen & 0xFF},profile={c['profile']},odr_hz={odr:.1f},bw_hz={bw:.1f},"
            f"valid_max_hz={vmax:.1f},alias_risk={alias},range_g={c['range_g']},"
            f"lsb_per_g={RANGES[c['range_g']]:.1f},baseline_tau_s={c['baseline_tau_s']:.3f},"
            f"rms_window_ms={c['rms_window_ms']},primary_axis={c['primary_axis']},"
            f"threshold={c['threshold']:.3f},hysteresis={c['hysteresis']:.3f},persist={c['persist']},"
            f"event_window_ms={c['event_window_ms']},release_hold_ms={c['release_hold_ms']},"
            f"refractory_ms={c['refractory_ms']},report_hz={c['report_hz']},stream={c['stream']},"
            f"fft_n={c['fft_n']},fft_interval_ms={c['fft_interval_ms']},fft_fmin_hz={c['fft_fmin_hz']:.2f},"
            + ",".join(f"ofs_{a}={c['ofs'][i]:.5f}" for i, a in enumerate("xyz")) + ","
            + ",".join(f"scl_{a}={c['scl'][i]:.5f}" for i, a in enumerate("xyz")) + ","
            + ",".join(f"tare_{a}={c['tare'][i]:.5f}" for i, a in enumerate("xyz")) + ",units=m/s2")

    def ack(self, cmd: str, ok: bool, msg: str) -> None:
        self._emit(f"ACK,{cmd},{'OK' if ok else 'ERR'},{msg}")

    # -- commands --------------------------------------------------------
    def command(self, line: str) -> None:
        parts = line.replace(",", " ").replace("=", " ").split()
        if not parts:
            return
        w = parts[0].upper()
        if w == "PING":
            self.ack(w, True, "PONG")
        elif w == "HELLO":
            self.hello()
        elif w == "GETCFG":
            self.send_cfg()
        elif w == "START":
            self.running = True
            self.ack(w, True, "acquisition running")
        elif w == "STOP":
            self.running = False
            self.ack(w, True, "acquisition stopped")
        elif w == "RESET":
            self.reset_stats()
            self.ack(w, True, "test statistics and impact count cleared")
        elif w == "CAL":
            ms = int(parts[1]) if len(parts) > 1 else 2000
            self._cal = dict(end=self.t_us + ms * 1000, sum=np.zeros(3), sumsq=np.zeros(3), n=0)
            self.ack(w, True, f"collecting {ms} ms - keep the sensor still")
        elif w == "TARE":
            # Same timing as the firmware: settle, then average the monitored axis
            self._tare = dict(start=self.t_us + 500e3, end=self.t_us + 1500e3, sum=0.0, sumsq=0.0, n=0)
            self.ack(w, True, "zeroing (command) - keep the sensor still for 1500 ms")
        elif w == "SAVE":
            self.ack(w, True, "configuration saved to flash")
        elif w == "DEFAULTS":
            self.cfg = {k: (list(v) if isinstance(v, list) else v) for k, v in DEFAULT_CFG.items()}
            self._reconfigure(True)
            self.ack(w, True, "defaults restored, flash copy erased")
        elif w == "SIM_ORIENT" and len(parts) > 1 and parts[1].upper() in ORIENT:
            self.gravity_dir = np.array(ORIENT[parts[1].upper()], dtype=float)
            self.bumps = False
            self.ack(w, True, parts[1].upper())
        elif w == "SIM_BUMPS" and len(parts) > 1:
            self.bumps = parts[1] == "1"
            self.ack(w, True, parts[1])
        elif w == "SET" and len(parts) >= 3:
            self._set(parts[1].upper(), parts[2])
        else:
            self.ack(w, False, "unknown command")

    def _set(self, key: str, val: str) -> None:
        c = self.cfg
        sensor = False
        try:
            if key == "PROFILE":
                v = val.upper()
                if v.isdigit():
                    v = PROFILE_NAMES[int(v)]
                if v not in PROFILES:
                    raise ValueError(f"unknown profile {val}")
                sensor = v != c["profile"]
                c["profile"] = v
            elif key == "RANGE":
                if int(val) not in RANGES:
                    raise ValueError("range must be 2, 4, 8 or 16")
                sensor = int(val) != c["range_g"]
                c["range_g"] = int(val)
            elif key == "AXIS":
                v = val.upper()
                v = "XYZ"[int(v)] if v.isdigit() else v
                if v not in ("X", "Y", "Z"):
                    raise ValueError("axis must be X, Y, Z or 0..2")
                c["primary_axis"] = v
            elif key == "FFT_N":
                n = int(val)
                if n < 256 or n > 2048 or n & (n - 1):
                    raise ValueError("fft_n must be a power of two in 256..2048")
                c["fft_n"] = n
            elif key[:5] == "TARE_" and key[5:] in ("X", "Y", "Z"):
                v = float(val)
                if not -200 <= v <= 200:
                    raise ValueError(f"{key} must be -200..200")
                c["tare"]["XYZ".index(key[5])] = v
            elif key[:4] in ("OFS_", "SCL_") and key[4:] in ("X", "Y", "Z"):
                v = float(val)
                i = "XYZ".index(key[4])
                if key.startswith("OFS"):
                    if not -20 <= v <= 20:
                        raise ValueError(f"{key} must be -20..20")
                    c["ofs"][i] = v
                else:
                    if not 0.8 <= v <= 1.2:
                        raise ValueError(f"{key} must be 0.8..1.2")
                    c["scl"][i] = v
            elif key in SET_KEYS:
                k, typ, lo, hi = SET_KEYS[key]
                v = typ(float(val)) if typ is int else float(val)
                if not lo <= v <= hi:
                    raise ValueError(f"{key} must be {lo}..{hi}")
                c[k] = v
            else:
                raise ValueError(f"unknown key {key}")
            if c["hysteresis"] >= c["threshold"]:
                raise ValueError("hysteresis must be below threshold")
        except ValueError as exc:
            self.ack("SET", False, str(exc))
            return
        self._reconfigure(sensor)
        self.ack("SET", True, "applied")

    def _reconfigure(self, sensor: bool) -> None:
        self._flush_stream()
        self.gen += 1
        keep_test = self.test
        self._init_dsp()
        if sensor:
            self.reset_stats()
            self.fft_count = 0
        else:
            self.test = keep_test
        self.send_cfg()

    # -- signal generation -----------------------------------------------
    def _signal(self, t: np.ndarray) -> np.ndarray:
        n = len(t)
        a = np.tile(self.gravity_dir * G, (n, 1))
        a += self.rng.normal(0, 0.04, (n, 3))
        if self.bumps:  # "vehicle running"; SIM_ORIENT puts the sensor on a quiet bench
            a[:, 0] += 0.15 * np.sin(2 * np.pi * 47.0 * t)
            a[:, 2] += 0.25 * np.sin(2 * np.pi * 47.0 * t + 0.4)
        if self.bumps and self._cal is None:
            while t[-1] >= self._next_bump:
                self._bumps.append((self._next_bump, self.rng.uniform(15, 45)))
                self._next_bump += self.rng.uniform(1.8, 3.5)
        fn, zeta = 14.0, 0.25
        wn = 2 * np.pi * fn
        wd = wn * math.sqrt(1 - zeta ** 2)
        keep = []
        for t0, amp in self._bumps:
            tt = t - t0
            m = tt >= 0
            if not m.any():
                keep.append((t0, amp))
                continue
            ts = tt[m]
            main = amp * np.exp(-zeta * wn * ts) * np.sin(wd * ts)
            hf = 0.35 * amp * np.exp(-60 * ts) * np.sin(2 * np.pi * 120 * ts)
            a[m, 2] += main + hf
            a[m, 0] += 0.12 * main
            a[m, 1] += -0.08 * main
            if ts[-1] < 1.5:
                keep.append((t0, amp))
        self._bumps = keep
        return a

    # -- per-block processing (port of acquisition.cpp/impact_detector.cpp) --
    def step(self, dt_s: float) -> List[str]:
        """Advance the simulation by dt_s seconds; returns the emitted lines."""
        if self.running:
            n = int(round(dt_s * self.fs_true + getattr(self, "_frac", 0.0)))
            self._frac = dt_s * self.fs_true + getattr(self, "_frac", 0.0) - n
            if n > 0:
                self._process(n)
        self.t_us += dt_s * 1e6
        self._report(dt_s)
        out, self.out = self.out, []
        return out

    def _process(self, n: int) -> None:
        c = self.cfg
        t = (self.idx + np.arange(n)) / self.fs_true
        a_true = self._signal(t)
        a_meas = a_true * self.true_scl + self.true_ofs     # sensor errors
        lsb = RANGES[c["range_g"]]
        counts = np.clip(np.round(a_meas / G * lsb), -32768, 32767).astype(np.int16)
        clip = np.any((counts == 32767) | (counts == -32768), axis=1)
        a_uncal = counts.astype(float) * (G / lsb)
        a = (a_uncal - np.array(c["ofs"])) * np.array(c["scl"]) - np.array(c["tare"])
        ax = "XYZ".index(c["primary_axis"])
        if self.base is None:
            self.base = a[0].copy()
        for k in range(n):
            self.base += self.alpha * (a[k] - self.base)
            d = a[k] - self.base
            dmag = float(np.sqrt(d @ d))
            dp = float(d[ax])
            v = abs(dp)  # vibration metrics use the primary axis only (as the firmware)
            self._window(v, dp)
            self._test(a[k], v, dp, bool(clip[k]))
            self.peak_int = max(self.peak_int, v)
            if self._tare is None:  # detection pauses while zeroing (as the firmware)
                self._detect(self.idx, v, d)
            tr = self._tare
            if tr is not None and self.t_us >= tr["start"]:
                v = float(a[k, ax] + c["tare"][ax])
                tr["sum"] += v
                tr["sumsq"] += v * v
                tr["n"] += 1
            self.fft_ring[self.idx % 4096] = dp
            self.fft_count += 1
            if self._cal is not None:
                self._cal["sum"] += a_uncal[k]
                self._cal["sumsq"] += a_uncal[k] ** 2
                self._cal["n"] += 1
            self.idx += 1
            self.last = (a[k], d, dmag)
        if c["stream"]:
            self.stream_buf.append(counts)
            if self.stream_first is None:
                self.stream_first = self.idx - n
            self._flush_stream(partial=False)
        if self._cal is not None and self.t_us >= self._cal["end"] and self._cal["n"] > 1:
            self._finish_cal()
        tr = self._tare
        if tr is not None and self.t_us >= tr["end"]:
            self._tare = None
            m = tr["sum"] / max(tr["n"], 1)
            sd = math.sqrt(max(tr["sumsq"] / max(tr["n"], 1) - m * m, 0))
            if tr["n"] < 20 or sd > 0.3:
                self.ack("TARE", False, f"zeroing refused: sensor moving - hold it still and press again (std {sd:.3f} m/s^2)")
            else:
                c["tare"][ax] = m
                self._reconfigure(False)
                self.ack("TARE", True, f"{c['primary_axis']} axis zeroed: tare {m:.4f} m/s^2 ({m / G:.4f} g), "
                                       f"std {sd:.4f}, {tr['n']} samples, saved to flash")

    def _flush_stream(self, partial: bool = True) -> None:
        if not self.stream_buf:
            return
        allc = np.concatenate(self.stream_buf)
        first = self.stream_first
        pos = 0
        while len(allc) - pos >= 50 or (partial and pos < len(allc)):
            blk = allc[pos:pos + 50]
            b64 = base64.b64encode(blk.astype("<i2").tobytes()).decode()
            self._emit(f"R,{self.gen & 0xFF},{first + pos},{len(blk)},{b64}")
            pos += len(blk)
        rest = allc[pos:]
        self.stream_buf = [rest] if len(rest) else []
        self.stream_first = first + pos if len(rest) else None

    def _window(self, dmag: float, dp: float) -> None:
        w = self.win
        if w["n"] == 0:
            w.update(pmax=dp, pmin=dp, peak=dmag)
        w["sumsq"] += dmag * dmag
        w["peak"] = max(w["peak"], dmag)
        w["pmax"] = max(w["pmax"], dp)
        w["pmin"] = min(w["pmin"], dp)
        w["n"] += 1
        if w["n"] >= self.win_n:
            self.win_res = dict(rms=math.sqrt(w["sumsq"] / w["n"]), p2p=w["pmax"] - w["pmin"])
            w.update(n=0, sumsq=0.0)

    def _test(self, a: np.ndarray, dmag: float, dp: float, clip: bool) -> None:
        s = self.test
        if s["samples"] == 0:
            s["max_raw"], s["min_raw"] = list(a), list(a)
            s["max_dp"] = s["min_dp"] = dp
        s["samples"] += 1
        s["sumsq"] += dmag * dmag
        s["max_dyn"] = max(s["max_dyn"], dmag)
        s["max_raw"] = [max(x, y) for x, y in zip(s["max_raw"], a)]
        s["min_raw"] = [min(x, y) for x, y in zip(s["min_raw"], a)]
        s["max_dp"] = max(s["max_dp"], dp)
        s["min_dp"] = min(s["min_dp"], dp)
        s["clipped"] += int(clip)

    def _detect(self, idx: int, dmag: float, d: np.ndarray) -> None:
        c, fs = self.cfg, self.odr
        persist = max(1, c["persist"])
        win = max(1, round(c["event_window_ms"] * fs / 1000))
        hold = max(1, round(c["release_hold_ms"] * fs / 1000))
        refr = round(c["refractory_ms"] * fs / 1000)
        thr, hyst = c["threshold"], c["hysteresis"]
        if self.det_state == "REFR":
            if idx < self.det_refr_until:
                return
            self.det_state, self.det_above = "IDLE", 0
        if self.det_state == "IDLE":
            if dmag > thr:
                if self.det_above == 0:
                    self.det_cur = dict(start=idx, peak=0.0, peak_idx=idx, vec=d.copy(), t=self.t_us)
                if dmag > self.det_cur["peak"]:
                    self.det_cur.update(peak=dmag, peak_idx=idx, vec=d.copy())
                self.det_above += 1
                if self.det_above >= persist:
                    self.det_state, self.det_below = "ACTIVE", 0
            else:
                self.det_above = 0
            return
        cur = self.det_cur
        if dmag > cur["peak"]:
            cur.update(peak=dmag, peak_idx=idx, vec=d.copy())
        self.det_below = self.det_below + 1 if dmag < thr - hyst else 0
        if self.det_below >= hold:
            self._finish_event(idx - self.det_below + 1, False)
        elif idx - cur["start"] + 1 >= win:
            self._finish_event(idx + 1, True)

    def _finish_event(self, end: int, trunc: bool) -> None:
        cur, fs = self.det_cur, self.odr
        self.impacts += 1
        dur = (end - cur["start"]) * 1000 / fs
        interval = (cur["start"] - self.det_prev_start) * 1000 / fs if self.det_prev_start is not None else -1
        self.det_prev_start = cur["start"]
        v = cur["vec"]
        self._emit(f"E,{self.impacts},{cur['start']},{cur['peak_idx']},{end},{cur['t'] / 1000:.3f},"
                   f"{cur['peak']:.4f},{v[0]:.4f},{v[1]:.4f},{v[2]:.4f},{dur:.2f},{interval:.2f},{int(trunc)}")
        self.det_refr_until = end + round(self.cfg["refractory_ms"] * fs / 1000)
        self.det_state = "REFR"
        self.det_above = self.det_below = 0

    def _finish_cal(self) -> None:
        cal = self._cal
        self._cal = None
        n = cal["n"]
        m = cal["sum"] / n
        sd = np.sqrt(np.maximum(cal["sumsq"] / n - m * m, 0))
        roll = math.degrees(math.atan2(m[1], m[2]))
        pitch = math.degrees(math.atan2(-m[0], math.hypot(m[1], m[2])))
        self._emit(f"CAL,{n},{m[0]:.4f},{m[1]:.4f},{m[2]:.4f},{sd[0]:.5f},{sd[1]:.5f},{sd[2]:.5f},"
                   f"{np.linalg.norm(m):.4f},{roll:.2f},{pitch:.2f},0,{self.fs_true:.2f}")

    def _report(self, dt_s: float) -> None:
        self._status_acc += dt_s
        self._test_acc += dt_s
        self._fft_acc += dt_s
        if self._status_acc < 1.0 / self.cfg["report_hz"]:
            return
        self._status_acc = 0.0
        a, d, dmag = getattr(self, "last", (np.zeros(3), np.zeros(3), 0.0))
        s = self.test
        rms = math.sqrt(s["sumsq"] / s["samples"]) if s["samples"] else 0.0
        p2p = s["max_dp"] - s["min_dp"] if s["samples"] else 0.0
        flags = 1 | (2 if self.running else 0)  # SENSOR_OK | RUNNING
        fs_meas = self.fs_true if self.idx > self.fs_true else 0.0
        if fs_meas and abs(fs_meas - self.odr) / self.odr > 0.03:
            flags |= 1 << 5
        if PROFILES[self.cfg["profile"]][3]:
            flags |= 1 << 7
        self._emit(
            f"S,{max(self.idx - 1, 0)},{self.t_us / 1000:.3f},{a[0]:.4f},{a[1]:.4f},{a[2]:.4f},"
            f"{np.linalg.norm(a):.4f},{d[0]:.4f},{d[1]:.4f},{d[2]:.4f},{dmag:.4f},"
            f"{self.win_res['rms']:.4f},{self.win_res['p2p']:.4f},{self.peak_int:.4f},{s['max_dyn']:.4f},"
            f"{rms:.4f},{p2p:.4f},{self.impacts},{int(self.det_state == 'ACTIVE')},{fs_meas:.2f},{flags},"
            f"0,0,0,0,0.000")
        self.peak_int = 0.0
        if self._test_acc >= 1.0:
            self._test_acc = 0.0
            self._emit(
                f"TS,{max(self.idx - 1, 0)},{s['samples']},{s['max_raw'][0]:.4f},{s['min_raw'][0]:.4f},"
                f"{s['max_raw'][1]:.4f},{s['min_raw'][1]:.4f},{s['max_raw'][2]:.4f},{s['min_raw'][2]:.4f},"
                f"{s['max_dyn']:.4f},{s['max_dp']:.4f},{s['min_dp']:.4f},{rms:.4f},{p2p:.4f},"
                f"{self.impacts},{s['clipped']}")
        if self._fft_acc * 1000 >= self.cfg["fft_interval_ms"] and self.running:
            self._fft_acc = 0.0
            self._fft(fs_meas)

    def _fft(self, fs_meas: float) -> None:
        from .analysis import amplitude_spectrum, dominant_peak
        n = self.cfg["fft_n"]
        if self.fft_count < n:
            return
        end = self.idx
        x = self.fft_ring[(np.arange(end - n, end)) % 4096]
        odr, _, vmax, alias = PROFILES[self.cfg["profile"]]
        fs = fs_meas or odr
        f, amp = amplitude_spectrum(x, fs, n)
        p = dominant_peak(f, amp[:, 0], self.cfg["fft_fmin_hz"], min(vmax, fs / 2))
        valid = int(p.valid and vmax > 0 and not alias and fs_meas > 0)
        self._emit(f"F,{end - 1},{n},{fs:.2f},{fs / n:.4f},{self.cfg['fft_fmin_hz']:.2f},{vmax:.2f},"
                   f"{p.freq_hz:.3f},{p.amp:.5f},{p.noise_floor:.5f},{valid}")

