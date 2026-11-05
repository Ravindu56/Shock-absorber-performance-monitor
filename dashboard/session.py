"""Monitoring session state, independent of the GUI.

``Session.handle(msg)`` consumes protocol messages; ``Session.tick()`` runs
deferred work (impact decay analysis once enough post-event samples have
arrived). The GUI only reads the state kept here, which also makes the whole
pipeline testable headlessly (see tests/test_pipeline.py).
"""

from __future__ import annotations

import collections
import time
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional

import numpy as np

from . import calibration
from .analysis import (DecayResult, HostProcessor, SampleBuffer, SamplingCheck, analyze_event,
                       noise_rms, sampling_check)
from .protocol import DeviceConfig, Message, describe_flags
from .recorder import Recorder, event_row


@dataclass
class Settings:
    """Dashboard-side (host) parameters. Device parameters live in the firmware."""
    buffer_s: float = 60.0            # history kept in memory
    analysis_ms: float = 1000.0       # ring-down window after the event peak
    pre_ms: float = 100.0             # samples kept before the event start (plots, noise)
    compression_sign: int = 1         # +1: positive primary-axis d = compression
    settle_frac: float = 0.10         # settling = |d| stays below this fraction of the peak
    analysis_timeout_s: float = 3.0   # analyse with what's available after this wall time
    max_events: int = 500             # events kept in the table


@dataclass
class EventRecord:
    ev: dict
    received: float                   # time.monotonic() at reception
    result: Optional[DecayResult] = None
    row: Dict[str, object] = field(default_factory=dict)

    @property
    def analysed(self) -> bool:
        return self.result is not None


class Session:
    def __init__(self, settings: Optional[Settings] = None, record_dir: str = "recordings") -> None:
        self.s = settings or Settings()
        self.processor = HostProcessor()
        self.buffer = SampleBuffer(int(self.s.buffer_s * 1000))
        self.recorder = Recorder(record_dir)
        self.cfg: Optional[DeviceConfig] = None
        self.hello: Optional[dict] = None
        self.status: Optional[dict] = None
        self.test: Optional[dict] = None
        self.spectrum: Optional[dict] = None
        self.last_cal: Optional[dict] = None
        self.events: List[EventRecord] = []
        self.log: Deque[str] = collections.deque(maxlen=2000)
        self.new_log: List[str] = []
        self.quiet_noise: Optional[float] = None   # primary-axis RMS when nothing happens
        self.status_count = 0
        self.raw_count = 0
        self.last_status_time = 0.0
        self.cal_listeners: List = []              # callables(cal_msg) for CAL results
        self.restarts = 0
        self._rec_from_idx = 0                     # events before this idx are not recorded

    # -- derived ---------------------------------------------------------
    @property
    def fs(self) -> float:
        if self.status and float(self.status.get("fs_meas", 0)) > 0:
            return float(self.status["fs_meas"])
        return self.cfg.odr_hz if self.cfg else 1000.0

    def sampling(self) -> SamplingCheck:
        return sampling_check(self.cfg, self.status, self.processor.received, self.processor.missing)

    def _log(self, text: str) -> None:
        line = time.strftime("%H:%M:%S ") + text
        self.log.append(line)
        self.new_log.append(line)

    # -- message handling ------------------------------------------------
    def handle(self, msg: Message) -> None:
        k = msg.kind
        if k == "R":
            self._on_raw(msg.data)
        elif k == "S":
            self.status = msg.data
            self.status_count += 1
            self.last_status_time = time.monotonic()
            self.recorder.write_status(msg.data, time.time())
        elif k == "TS":
            self.test = msg.data
        elif k == "E":
            self._on_event(msg.data)
        elif k == "F":
            self.spectrum = msg.data
            self.recorder.write_spectrum(msg.data)
        elif k == "CFG":
            first = self.cfg is None
            self.cfg = msg.data
            self.processor.set_config(msg.data)
            if first:
                self._log(f"config: profile {self.cfg.get('profile')}, {self.cfg.odr_hz:.2f} Hz, "
                          f"+-{self.cfg.get('range_g')} g, threshold {self.cfg.get('threshold')} m/s^2")
        elif k == "HELLO":
            self.hello = msg.data
            d = msg.data
            self._log(f"HELLO firmware {d['firmware']} protocol {d['protocol']}, sensor {d['address']} "
                      f"WHO_AM_I {d['who_am_i']}, status {d['status']}")
            if d["protocol"] != 1:
                self._log("WARNING: protocol version mismatch - update dashboard or firmware")
        elif k == "CAL":
            self.last_cal = msg.data
            self._log("CAL: " + calibration.stationary_report(
                msg.data, self.cfg.offset if self.cfg else np.zeros(3),
                self.cfg.scale if self.cfg else np.ones(3)).text)
            for cb in list(self.cal_listeners):
                cb(msg.data)
        elif k == "ACK":
            d = msg.data
            self._log(f"{d['cmd']}: {'OK' if d['ok'] else 'FAILED'} {d['msg']}")
        elif k == "ERR":
            self._log(f"DEVICE ERROR {msg.data['code']}: {msg.data['msg']}")
        elif k == "LOG":
            self._log("device: " + msg.data)
        elif k == "LINK":
            self._log("LINK: " + msg.data)
        elif k == "UNKNOWN" and msg.line:
            self._log("unparsed: " + msg.line[:120])

    def _on_raw(self, blk) -> None:
        if self.cfg is None or (blk.gen & 0xFF) != (self.cfg.gen & 0xFF):
            return  # samples of a configuration we have no CFG for (scale unknown)
        idx, a, d, restarted = self.processor.process(blk.first_idx, blk.counts)
        if restarted:
            self.restarts += 1
            self.buffer.clear()
            self._log("sample index went backwards - device restarted; buffer cleared")
        self.buffer.append(idx, a, d, blk.counts)
        self.raw_count += len(idx)
        self.recorder.write_samples(idx, a, d, blk.counts, self.fs, self.cfg.primary_axis)
        # Quiet-noise estimate: blocks far below the impact threshold.
        ax = self.cfg.primary_axis
        thr = float(self.cfg.get("threshold", 9.81))
        if np.max(np.linalg.norm(d, axis=1)) < 0.25 * thr:
            r = noise_rms(d[:, ax])
            self.quiet_noise = r if self.quiet_noise is None else 0.95 * self.quiet_noise + 0.05 * r

    def _on_event(self, ev: dict) -> None:
        self.events.append(EventRecord(ev, time.monotonic()))
        if len(self.events) > self.s.max_events:
            self.events.pop(0)

    # -- deferred analysis -----------------------------------------------
    def tick(self) -> List[EventRecord]:
        done = []
        last = self.buffer.last_idx
        now = time.monotonic()
        for rec in self.events:
            if rec.analysed:
                continue
            need = int(rec.ev["peak_idx"]) + int(self.s.analysis_ms * self.fs / 1000)
            if (last is not None and last >= need) or now - rec.received > self.s.analysis_timeout_s:
                self._analyse(rec)
                done.append(rec)
        return done

    def _analyse(self, rec: EventRecord) -> None:
        ev, fs = rec.ev, self.fs
        pre = int(self.s.pre_ms * fs / 1000)
        i0 = int(ev["start_idx"]) - pre
        i1 = int(ev["peak_idx"]) + int(self.s.analysis_ms * fs / 1000) + 1
        idx, a, d = self.buffer.range(i0, i1)
        notes: List[str] = []
        res: Optional[DecayResult] = None
        ax = self.cfg.primary_axis if self.cfg else 2
        if len(idx) == 0:
            notes.append("no raw samples for this event (stream off or buffer overwritten)")
        else:
            if np.any(np.diff(idx) != 1):
                notes.append("stream gap inside the event window - decay analysis skipped")
            elif idx[-1] < i1 - 1:
                notes.append("analysis window incomplete")
            peak_i = int(ev["peak_idx"]) - int(idx[0])
            if not notes or notes == ["analysis window incomplete"]:
                # Reference = static acceleration just before the impact. The EMA
                # baseline used for live d absorbs part of the ring-down's own mean
                # and would leave a decaying offset that skews the late, small lobes.
                n_pre = max(0, int(ev["start_idx"]) - int(idx[0]))
                if n_pre >= 20:
                    x = a[:, ax].astype(np.float64) - float(np.mean(a[:n_pre, ax]))
                    nz = noise_rms(x[:n_pre])
                else:
                    x = d[:, ax].astype(np.float64)
                    nz = self.quiet_noise or 0.05
                    notes.append("no pre-event samples - analysed EMA-filtered d")
                if self.quiet_noise is not None:
                    nz = min(nz, self.quiet_noise)
                vmax = float(self.cfg.get("valid_max_hz", 0)) if self.cfg else 0.0
                if not self.sampling().spectral_ok:
                    vmax = 0.0  # frequencies are not validated in this configuration
                res = analyze_event(x, fs, peak_i, nz, self.s.compression_sign,
                                    self.s.settle_frac, vmax)
                res.notes = notes + res.notes
        if res is None:
            res = DecayResult(notes=notes)
        rec.result = res
        rec.row = event_row(ev, res, fs)
        if self.recorder.active and int(ev["start_idx"]) >= self._rec_from_idx:
            self.recorder.write_event(rec.row)

    # -- recording -------------------------------------------------------
    def metadata(self) -> Dict[str, object]:
        m: Dict[str, object] = {}
        if self.hello:
            m.update({f"device_{k}": v for k, v in self.hello.items()})
        if self.cfg:
            m.update({f"cfg_{k}": v for k, v in self.cfg.values.items()})
        m.update({"host_compression_sign": self.s.compression_sign,
                  "host_analysis_ms": self.s.analysis_ms, "host_settle_frac": self.s.settle_frac})
        return m

    def final_stats(self) -> Dict[str, object]:
        out: Dict[str, object] = {}
        sc = self.sampling()
        out.update({"fs_nominal_hz": sc.odr_nominal, "fs_measured_hz": sc.fs_measured,
                    "fs_deviation_pct": sc.deviation_pct, "dlpf_bw_hz": sc.dlpf_bw,
                    "valid_max_hz": sc.valid_max, "spectral_ok": int(sc.spectral_ok),
                    "missing_pct": sc.missing_pct, "sampling_verdict": sc.verdict})
        if self.test:
            out.update({f"test_{k}": v for k, v in self.test.items()})
        if self.status:
            for k in ("fifo_ovf", "i2c_err", "stream_drop", "gaps"):
                out[f"device_{k}"] = self.status.get(k)
            out["last_flags"] = "|".join(describe_flags(int(self.status.get("flags", 0))))
        if self.spectrum:
            out.update({"last_dom_hz": self.spectrum.get("dom_hz"),
                        "last_dom_valid": self.spectrum.get("valid")})
        return out

    def start_recording(self, label: str) -> str:
        path = self.recorder.start(label, self.metadata())
        self.processor.reset_counters()
        self._rec_from_idx = (self.buffer.last_idx or -1) + 1
        self._log(f"recording to {path}")
        return path

    def stop_recording(self) -> Optional[str]:
        for rec in self.events:   # analyse what is still pending so it lands in the file
            if not rec.analysed:
                self._analyse(rec)
        path = self.recorder.stop(self.final_stats())
        if path:
            self._log(f"recording saved: {path}")
        return path

    def export_snapshot(self, label: str) -> str:
        """Write the in-memory history (samples + analysed events) as a recording folder."""
        rec = Recorder(self.recorder.base_dir)
        path = rec.start(label + "_snapshot", self.metadata())
        idx, a, d, c = self.buffer.latest(with_counts=True)
        rec.write_samples(idx, a, d, c, self.fs, self.cfg.primary_axis if self.cfg else 2)
        for e in self.events:
            if e.analysed:
                rec.write_event(e.row)
        rec.stop(self.final_stats())
        self._log(f"snapshot exported: {path}")
        return path

