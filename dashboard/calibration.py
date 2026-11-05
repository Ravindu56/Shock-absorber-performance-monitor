"""Accelerometer calibration from stationary CAL measurements.

6-position method: hold the sensor still with each axis pointing straight up
and then straight down. An axis pointing up reads +1 g, down reads -1 g, so
for axis i with uncalibrated means p (up) and m (down):

    offset_i = (p + m) / 2              [m/s^2]
    scale_i  = 2 g / (p - m)

and the firmware applies  a_cal = (a - offset) * scale.  The CAL results the
firmware sends are always uncalibrated, so re-running the procedure never
compounds an earlier calibration.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .protocol import G

POSITIONS = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
# m/s^2 per axis; above this the sensor was moving. MPU6500 noise is ~300 ug/sqrt(Hz),
# i.e. ~0.055 m/s^2 RMS at the 218 Hz bandwidth, so this leaves margin for real units.
MAX_STATIONARY_STD = 0.15
MAX_OFF_AXIS_FRAC = 0.15      # off-axis components must be < 15 % of g (orientation check)


@dataclass
class CalCapture:
    mean: np.ndarray
    std: np.ndarray
    samples: int


@dataclass
class CalSolution:
    offset: np.ndarray
    scale: np.ndarray
    residual_g: Dict[str, float]          # |a_cal| - 1 g in each position, in m/s^2
    warnings: List[str] = field(default_factory=list)

    def commands(self) -> List[str]:
        cmds = [f"SET OFS_{a} {self.offset[i]:.5f}" for i, a in enumerate("XYZ")]
        cmds += [f"SET SCL_{a} {self.scale[i]:.5f}" for i, a in enumerate("XYZ")]
        return cmds


def check_capture(pos: str, cap: CalCapture) -> List[str]:
    """Problems with one capture (empty list if it looks good)."""
    warn = []
    if np.any(cap.std > MAX_STATIONARY_STD):
        warn.append(f"{pos}: sensor not still (std up to {cap.std.max():.2f} m/s^2)")
    axis = "XYZ".index(pos[1])
    sign = 1 if pos[0] == "+" else -1
    if np.sign(cap.mean[axis]) != sign or abs(cap.mean[axis]) < 0.8 * G:
        warn.append(f"{pos}: {pos[1]} axis reads {cap.mean[axis]:+.2f} m/s^2 - wrong orientation?")
    off = [cap.mean[j] for j in range(3) if j != axis]
    if max(abs(v) for v in off) > MAX_OFF_AXIS_FRAC * G:
        warn.append(f"{pos}: not level (off-axis {max(abs(v) for v in off):.2f} m/s^2)")
    return warn


def solve(caps: Dict[str, CalCapture]) -> Optional[CalSolution]:
    if any(p not in caps for p in POSITIONS):
        return None
    offset = np.zeros(3)
    scale = np.ones(3)
    warnings: List[str] = []
    for p in POSITIONS:
        warnings += check_capture(p, caps[p])
    for i, a in enumerate("XYZ"):
        up, dn = caps["+" + a].mean[i], caps["-" + a].mean[i]
        if up - dn <= 0:
            warnings.append(f"{a}: +{a} reading not above -{a} reading - captures swapped?")
            continue
        offset[i] = 0.5 * (up + dn)
        scale[i] = 2.0 * G / (up - dn)
    if np.any(np.abs(scale - 1) > 0.1):
        warnings.append("scale correction > 10 % - check captures")
    residual = {p: float(np.linalg.norm((caps[p].mean - offset) * scale) - G) for p in POSITIONS}
    return CalSolution(offset, np.clip(scale, 0.8, 1.2), residual, warnings)


@dataclass
class StationaryReport:
    mag_cal: float          # |a| after applying the device calibration [m/s^2]
    mag_error_pct: float
    noise_mg: np.ndarray    # per-axis std in milli-g
    roll_deg: float
    pitch_deg: float
    ok: bool
    text: str


def stationary_report(cal_msg: dict, offset: np.ndarray, scale: np.ndarray) -> StationaryReport:
    mean = np.array([cal_msg["mean_x"], cal_msg["mean_y"], cal_msg["mean_z"]], dtype=float)
    std = np.array([cal_msg["std_x"], cal_msg["std_y"], cal_msg["std_z"]], dtype=float)
    a = (mean - offset) * scale
    mag = float(np.linalg.norm(a))
    err = 100.0 * (mag - G) / G
    noise = std * scale / G * 1000.0
    ok = abs(err) < 2.0 and np.all(std < MAX_STATIONARY_STD) and int(cal_msg.get("clipped", 0)) == 0
    roll = float(np.degrees(np.arctan2(a[1], a[2])))
    pitch = float(np.degrees(np.arctan2(-a[0], np.hypot(a[1], a[2]))))
    text = (f"|g| = {mag:.2f} m/s^2 ({err:+.2f} % vs 9.80665)   noise X/Y/Z = "
            f"{noise[0]:.2f} / {noise[1]:.2f} / {noise[2]:.2f} mg RMS   roll {roll:+.2f} deg  "
            f"pitch {pitch:+.2f} deg   {int(cal_msg['samples'])} samples @ {cal_msg['fs_meas']:.2f} Hz")
    if not ok:
        text += "   -> " + ("moved during capture" if np.any(std >= MAX_STATIONARY_STD) else
                             "|g| error > 2 %: run the 6-position calibration")
    return StationaryReport(mag, err, noise, roll, pitch, bool(ok), text)
