"""Parser for the firmware's line protocol (see README.md, "Serial protocol").

Every device line is ``TYPE,field,field,...``. Column names for S/TS/E/F/CAL
lines come from the ``H`` header lines the firmware sends after HELLO; the
defaults below mirror ``src/serial_protocol.cpp`` so that a dashboard that
connects mid-stream (and missed the headers) still decodes correctly.
"""

from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

PROTOCOL_VERSION = 1
G = 9.80665  # m/s^2 per g

# Must match proto::sendHeaders() in src/serial_protocol.cpp.
DEFAULT_HEADERS: Dict[str, List[str]] = {
    "S": ("idx,t_ms,ax,ay,az,amag,dx,dy,dz,dmag,rms_win,p2p_win,peak_int,test_max_dyn,test_rms,"
          "test_p2p,impacts,impact_active,fs_meas,flags,fifo_ovf,i2c_err,stream_drop,gaps,"
          "dup_ratio").split(","),
    "TS": ("idx,samples,max_x,min_x,max_y,min_y,max_z,min_z,max_dyn,max_dprim,min_dprim,rms,p2p,"
           "impacts,clipped").split(","),
    "E": ("id,start_idx,peak_idx,end_idx,t_start_ms,peak_dyn,peak_dx,peak_dy,peak_dz,duration_ms,"
          "interval_ms,truncated").split(","),
    "F": "idx,n,fs,df,fmin,fmax,dom_hz,dom_amp,noise_floor,valid".split(","),
    "CAL": ("samples,mean_x,mean_y,mean_z,std_x,std_y,std_z,mag_mean,roll_deg,pitch_deg,clipped,"
            "fs_meas").split(","),
}

# Status flag bits (StatusFlag in src/acquisition.h).
FLAGS = {
    0: ("SENSOR_OK", "sensor initialised and responding"),
    1: ("RUNNING", "acquisition running"),
    2: ("FIFO_OVF", "FIFO overflow - samples lost"),
    3: ("I2C_ERR", "I2C transaction failed"),
    4: ("CLIP", "sample at full scale - increase RANGE"),
    5: ("RATE_DEV", "measured sample rate deviates > 3 % from nominal"),
    6: ("STREAM_DROP", "raw stream buffer full - samples not transmitted"),
    7: ("ALIAS_RISK", "profile has no validated spectral band"),
    8: ("RECOVERING", "re-initialising I2C bus / sensor"),
    9: ("DUPLICATES", "> 20 % identical samples - sensor not updating?"),
    10: ("GAP", "sample sequence discontinuity"),
}
FLAG_SENSOR_OK, FLAG_RUNNING = 1 << 0, 1 << 1
FLAG_RATE_DEV, FLAG_ALIAS_RISK = 1 << 5, 1 << 7
# Bits that indicate a problem (shown in red when set).
FLAG_PROBLEMS = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)


def describe_flags(flags: int) -> List[str]:
    return [name for bit, (name, _) in FLAGS.items() if flags & (1 << bit)]


SENSOR_PROFILES = ["STD_1K_218", "LN_1K_99", "LN_500_45", "EXP_1K_420"]
RANGES_G = [2, 4, 8, 16]
AXES = "XYZ"


@dataclass
class Message:
    kind: str                 # HELLO, CFG, S, TS, E, F, R, CAL, ACK, ERR, LOG, H, UNKNOWN, LINK
    data: Any = None
    line: str = ""


@dataclass
class RawBlock:
    gen: int
    first_idx: int
    counts: np.ndarray        # shape (n, 3), int16 raw sensor counts (X, Y, Z)


@dataclass
class DeviceConfig:
    """Active firmware configuration, from a CFG line."""
    values: Dict[str, Any] = field(default_factory=dict)

    def __getitem__(self, k: str) -> Any:
        return self.values[k]

    def get(self, k: str, default: Any = None) -> Any:
        return self.values.get(k, default)

    @property
    def gen(self) -> int:
        return int(self.values.get("gen", 0))

    @property
    def odr_hz(self) -> float:
        return float(self.values.get("odr_hz", 1000.0))

    @property
    def lsb_per_g(self) -> float:
        return float(self.values.get("lsb_per_g", 2048.0))

    @property
    def primary_axis(self) -> int:
        v = str(self.values.get("primary_axis", "Z")).upper()
        return AXES.index(v) if v in AXES else 2

    @property
    def offset(self) -> np.ndarray:
        return np.array([float(self.values.get(f"ofs_{a}", 0.0)) for a in "xyz"])

    @property
    def scale(self) -> np.ndarray:
        return np.array([float(self.values.get(f"scl_{a}", 1.0)) for a in "xyz"])

    @property
    def tare(self) -> np.ndarray:
        """Zero offsets subtracted after calibration (button / TARE command)."""
        return np.array([float(self.values.get(f"tare_{a}", 0.0)) for a in "xyz"])


def _num(s: str) -> Any:
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


class Parser:
    """Stateful line parser (tracks H header lines)."""

    def __init__(self) -> None:
        self.headers: Dict[str, List[str]] = {k: list(v) for k, v in DEFAULT_HEADERS.items()}
        self.errors = 0
        self.last_error = ""

    def parse(self, line: str) -> Optional[Message]:
        line = line.strip()
        if not line:
            return None
        try:
            return self._parse(line)
        except (ValueError, IndexError, binascii.Error) as exc:
            self.errors += 1
            self.last_error = f"{exc}: {line[:80]}"
            return Message("UNKNOWN", str(exc), line)

    def _parse(self, line: str) -> Message:
        if line.startswith("#"):
            return Message("LOG", line[1:].strip(), line)
        kind, _, rest = line.partition(",")
        if kind == "R":
            gen, first, n, b64 = rest.split(",", 3)
            buf = base64.b64decode(b64, validate=True)
            n = int(n)
            if len(buf) != 6 * n:
                raise ValueError(f"R block length {len(buf)} != 6*{n}")
            counts = np.frombuffer(buf, dtype="<i2").reshape(n, 3).astype(np.int16)
            return Message("R", RawBlock(int(gen), int(first), counts), "")
        fields = rest.split(",") if rest else []
        if kind == "H":
            self.headers[fields[0]] = fields[1:]
            return Message("H", fields[0], line)
        if kind in self.headers:
            names = self.headers[kind]
            if len(fields) != len(names):
                raise ValueError(f"{kind} has {len(fields)} fields, expected {len(names)}")
            return Message(kind, {k: _num(v) for k, v in zip(names, fields)}, line)
        if kind == "CFG":
            vals = {}
            for kv in fields:
                k, _, v = kv.partition("=")
                vals[k] = _num(v)
            return Message("CFG", DeviceConfig(vals), line)
        if kind == "HELLO":
            proto, fw, addr, who, status = fields[:5]
            return Message("HELLO", {"protocol": int(proto), "firmware": fw, "address": addr,
                                     "who_am_i": who, "status": status}, line)
        if kind == "ACK":
            cmd, ok = fields[0], fields[1] == "OK"
            return Message("ACK", {"cmd": cmd, "ok": ok, "msg": ",".join(fields[2:])}, line)
        if kind == "ERR":
            return Message("ERR", {"code": fields[0], "msg": ",".join(fields[1:])}, line)
        return Message("UNKNOWN", None, line)
