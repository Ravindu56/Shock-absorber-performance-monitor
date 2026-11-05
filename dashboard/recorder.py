"""CSV recording of test sessions.

A recording is a folder ``<base>/<YYYYmmdd_HHMMSS>_<label>/`` containing

  samples.csv   every received sample of the monitored axis (Z): calibrated
                and dynamic acceleration (m/s^2 and g) plus the raw counts
  status.csv    the device's S lines (window RMS/P2P/peak, flags, counters)
  events.csv    impact events with compression/rebound and decay metrics
  spectra.csv   the device's dominant-frequency results (F lines)
  summary.csv   key,value: configuration, sampling verification, test
                statistics and event-metric means - the file compare.py reads
"""

from __future__ import annotations

import csv
import datetime as _dt
import math
import os
import re
from typing import Dict, Iterable, List, Optional

import numpy as np

from .analysis import DecayResult
from .protocol import G

# Only the monitored axis (Z by default) is recorded. Column names use "z"; the
# actual axis is stored as cfg_primary_axis in summary.csv.
SAMPLE_COLUMNS = ["idx", "t_s", "az_ms2", "az_g", "dz_ms2", "dz_g", "raw_z"]

EVENT_COLUMNS = ["id", "t_start_s", "start_idx", "peak_idx", "end_idx", "peak_dyn_ms2", "peak_dyn_g",
                 "duration_ms", "interval_ms", "truncated",
                 "compression_peak_ms2", "rebound_peak_ms2", "rebound_ratio", "comp_to_rebound_ms",
                 "lobes", "damped_freq_hz", "freq_valid", "decay_rate_per_s", "damping_ratio",
                 "log_decrement", "fit_r2", "settling_ms", "method", "notes"]

# Event metrics summarised (mean/std/min/max) in summary.csv.
SUMMARY_METRICS = ["peak_dyn_ms2", "compression_peak_ms2", "rebound_peak_ms2", "rebound_ratio",
                   "damped_freq_hz", "damping_ratio", "log_decrement", "settling_ms", "duration_ms"]


def event_row(ev: dict, res: Optional[DecayResult], fs: float) -> Dict[str, object]:
    row: Dict[str, object] = {
        "id": ev["id"], "t_start_s": ev["start_idx"] / fs if fs else "", "start_idx": ev["start_idx"],
        "peak_idx": ev["peak_idx"], "end_idx": ev["end_idx"], "peak_dyn_ms2": ev["peak_dyn"],
        "peak_dyn_g": ev["peak_dyn"] / G, "duration_ms": ev["duration_ms"], "interval_ms": ev["interval_ms"],
        "truncated": ev["truncated"],
    }
    if res is not None:
        row.update({
            "compression_peak_ms2": res.compression_peak, "rebound_peak_ms2": res.rebound_peak,
            "rebound_ratio": res.rebound_ratio, "comp_to_rebound_ms": res.comp_to_rebound_ms,
            "lobes": res.lobes, "damped_freq_hz": res.damped_freq_hz, "freq_valid": int(res.freq_valid),
            "decay_rate_per_s": res.decay_rate_per_s, "damping_ratio": res.damping_ratio,
            "log_decrement": res.log_decrement, "fit_r2": res.fit_r2, "settling_ms": res.settling_ms,
            "method": res.method, "notes": "; ".join(res.notes),
        })
    return {k: _fmt(v) for k, v in row.items()}


def _fmt(v: object) -> object:
    if isinstance(v, (float, np.floating)):
        return "" if not math.isfinite(float(v)) else f"{float(v):.6g}"
    return v


def summarize_events(rows: Iterable[Dict[str, object]]) -> Dict[str, object]:
    rows = list(rows)
    out: Dict[str, object] = {"events": len(rows)}
    for m in SUMMARY_METRICS:
        vals = []
        for r in rows:
            try:
                v = float(r.get(m, ""))
            except (TypeError, ValueError):
                continue
            if math.isfinite(v):
                vals.append(v)
        if vals:
            a = np.array(vals)
            out.update({f"{m}_mean": f"{a.mean():.6g}", f"{m}_std": f"{a.std(ddof=1) if len(a) > 1 else 0:.6g}",
                        f"{m}_min": f"{a.min():.6g}", f"{m}_max": f"{a.max():.6g}", f"{m}_n": len(a)})
    return out


class _Csv:
    def __init__(self, path: str, columns: List[str]) -> None:
        self.f = open(path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=columns, extrasaction="ignore")
        self.w.writeheader()
        self.columns = columns

    def write(self, row: Dict[str, object]) -> None:
        self.w.writerow(row)

    def close(self) -> None:
        self.f.close()


class Recorder:
    def __init__(self, base_dir: str) -> None:
        self.base_dir = base_dir
        self.path: Optional[str] = None
        self._files: Dict[str, _Csv] = {}
        self._samples_f = None
        self.samples = 0
        self.events: List[Dict[str, object]] = []
        self.meta: Dict[str, object] = {}
        self.started: Optional[_dt.datetime] = None

    @property
    def active(self) -> bool:
        return self.path is not None

    def start(self, label: str, meta: Dict[str, object]) -> str:
        self.stop({})
        self.started = _dt.datetime.now()
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", label.strip()) or "test"
        self.path = os.path.join(self.base_dir, f"{self.started:%Y%m%d_%H%M%S}_{safe}")
        os.makedirs(self.path, exist_ok=True)
        self._samples_f = open(os.path.join(self.path, "samples.csv"), "w", newline="", encoding="utf-8")
        self._samples_f.write(",".join(SAMPLE_COLUMNS) + "\n")
        self.samples = 0
        self.events = []
        self.meta = dict(meta, label=label, started=self.started.isoformat(timespec="seconds"))
        return self.path

    def write_samples(self, idx: np.ndarray, a: np.ndarray, d: np.ndarray, counts: np.ndarray, fs: float,
                      axis: int = 2) -> None:
        if not self._samples_f or len(idx) == 0:
            return
        t = idx / fs if fs else np.zeros(len(idx))
        az, dz = a[:, axis], d[:, axis]
        block = np.column_stack([idx, t, az, az / G, dz, dz / G, counts[:, axis]])
        fmt = ["%d", "%.6f", "%.5f", "%.6f", "%.5f", "%.6f", "%d"]
        np.savetxt(self._samples_f, block, fmt=fmt, delimiter=",")
        self.samples += len(idx)

    def _csv(self, name: str, columns: List[str]) -> Optional[_Csv]:
        if not self.path:
            return None
        if name not in self._files:
            self._files[name] = _Csv(os.path.join(self.path, name), columns)
        return self._files[name]

    def write_status(self, s: dict, host_time: float) -> None:
        f = self._csv("status.csv", ["host_time"] + list(s.keys()))
        if f:
            f.write(dict(s, host_time=f"{host_time:.3f}"))

    def write_spectrum(self, fmsg: dict) -> None:
        f = self._csv("spectra.csv", list(fmsg.keys()))
        if f:
            f.write(fmsg)

    def write_event(self, row: Dict[str, object]) -> None:
        f = self._csv("events.csv", EVENT_COLUMNS)
        if f:
            f.write(row)
            self.events.append(row)

    def flush(self) -> None:
        if self._samples_f:
            self._samples_f.flush()
        for f in self._files.values():
            f.f.flush()

    def stop(self, final: Dict[str, object]) -> Optional[str]:
        """Close files and write summary.csv. Returns the folder path."""
        if not self.path:
            return None
        path = self.path
        ended = _dt.datetime.now()
        summary: Dict[str, object] = dict(self.meta)
        summary.update(final)
        summary.update(ended=ended.isoformat(timespec="seconds"),
                       duration_s=f"{(ended - self.started).total_seconds():.1f}",
                       samples_recorded=self.samples)
        summary.update(summarize_events(self.events))
        write_summary(os.path.join(path, "summary.csv"), summary)
        if self._samples_f:
            self._samples_f.close()
        for f in self._files.values():
            f.close()
        self._files.clear()
        self._samples_f = None
        self.path = None
        return path


def write_summary(path: str, summary: Dict[str, object]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        for k, v in summary.items():
            w.writerow([k, _fmt(v)])


def read_summary(path: str) -> Dict[str, str]:
    with open(path, newline="", encoding="utf-8") as f:
        return {row["key"]: row["value"] for row in csv.DictReader(f)}
