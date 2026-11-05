"""Host-side signal processing (NumPy).

* ``HostProcessor``  raw counts -> calibrated acceleration a and dynamic
  acceleration d, using exactly the firmware's baseline filter so host and
  device numbers agree.
* ``SampleBuffer``   ring buffer of the last N seconds indexed by sample idx.
* ``amplitude_spectrum`` / ``dominant_peak``  Welch-averaged spectrum.
* ``sampling_check`` decides whether frequencies may be reported at all.
* ``analyze_event``  compression / rebound peaks and vibration decay
  (damped frequency, decay rate, damping ratio, settling time).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .protocol import G, DeviceConfig

TARGET_BAND_HZ = (500.0, 2000.0)  # the project's requested vibration band


# ---------------------------------------------------------------------------
# Raw counts -> acceleration
# ---------------------------------------------------------------------------
class HostProcessor:
    """Converts R blocks to calibrated a and baseline-removed d (both m/s^2).

    The baseline is the same first-order low-pass as dsp::BaselineFilter:
    b[k] = b[k-1] + alpha*(a[k] - b[k-1]), alpha = 1 - exp(-1/(fs*tau)),
    initialised to the first sample. It restarts on any gap or config change.
    """

    def __init__(self) -> None:
        self.cfg: Optional[DeviceConfig] = None
        self.alpha = 0.002
        self.b: Optional[np.ndarray] = None
        self.next_idx: Optional[int] = None
        self.missing = 0        # samples never received (stream drops, overflows, restarts)
        self.received = 0
        self._L_cache: dict = {}

    def set_config(self, cfg: DeviceConfig) -> None:
        old = self.cfg
        self.cfg = cfg
        fs = cfg.odr_hz
        tau = float(cfg.get("baseline_tau_s", 0.5))
        self.alpha = 1.0 - math.exp(-1.0 / (fs * tau)) if fs > 0 and tau > 0 else 1.0
        self._L_cache.clear()
        if (old is None or old.get("profile") != cfg.get("profile") or old.get("range_g") != cfg.get("range_g")
                or not np.array_equal(old.offset, cfg.offset) or not np.array_equal(old.scale, cfg.scale)
                or not np.array_equal(old.tare, cfg.tare)):
            self.b = None  # new scale / rate / zero: restart filter (the firmware does the same)

    def reset_counters(self) -> None:
        self.missing = 0
        self.received = 0

    def _lower(self, n: int) -> np.ndarray:
        L = self._L_cache.get(n)
        if L is None:
            k = np.arange(n)
            e = k[:, None] - k[None, :]
            L = np.where(e >= 0, (1.0 - self.alpha) ** np.clip(e, 0, None), 0.0)
            self._L_cache[n] = L
        return L

    def process(self, first_idx: int, counts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
        """Returns (idx, a, d, restarted). restarted=True if the index went backwards
        (device reset) — callers should clear their buffers."""
        cfg = self.cfg
        lsb = cfg.lsb_per_g if cfg else 2048.0
        a_uncal = counts.astype(np.float64) * (G / lsb)
        if cfg:
            a = (a_uncal - cfg.offset) * cfg.scale - cfg.tare
        else:
            a = a_uncal
        n = len(a)
        idx = np.arange(first_idx, first_idx + n, dtype=np.int64)

        restarted = False
        if self.next_idx is not None:
            if first_idx < self.next_idx - 1:
                restarted = True
                self.b = None
            elif first_idx > self.next_idx:
                self.missing += first_idx - self.next_idx
                self.b = None  # don't filter across a hole
        self.next_idx = first_idx + n
        self.received += n

        if self.b is None:
            self.b = a[0].copy()
        q = 1.0 - self.alpha
        p = q ** np.arange(1, n + 1)
        base = p[:, None] * self.b[None, :] + self.alpha * (self._lower(n) @ a)
        self.b = base[-1].copy()
        d = a - base
        return idx, a.astype(np.float32), d.astype(np.float32), restarted


# ---------------------------------------------------------------------------
# Ring buffer
# ---------------------------------------------------------------------------
class SampleBuffer:
    """Fixed-capacity ring of (idx, a, d, raw counts), oldest overwritten first."""

    def __init__(self, capacity: int) -> None:
        self.cap = int(capacity)
        self.idx = np.zeros(self.cap, dtype=np.int64)
        self.a = np.zeros((self.cap, 3), dtype=np.float32)
        self.d = np.zeros((self.cap, 3), dtype=np.float32)
        self.c = np.zeros((self.cap, 3), dtype=np.int16)
        self.n = 0      # valid samples
        self.head = 0   # next write position

    def clear(self) -> None:
        self.n = 0
        self.head = 0

    def append(self, idx: np.ndarray, a: np.ndarray, d: np.ndarray, counts: np.ndarray) -> None:
        m = len(idx)
        if m >= self.cap:
            idx, a, d, counts = idx[-self.cap:], a[-self.cap:], d[-self.cap:], counts[-self.cap:]
            m = self.cap
        pos = (self.head + np.arange(m)) % self.cap
        self.idx[pos], self.a[pos], self.d[pos], self.c[pos] = idx, a, d, counts
        self.head = (self.head + m) % self.cap
        self.n = min(self.n + m, self.cap)

    def latest(self, count: Optional[int] = None, with_counts: bool = False):
        """Oldest-first copies of the newest `count` samples: (idx, a, d[, counts])."""
        count = self.n if count is None else max(0, min(int(count), self.n))
        order = (self.head - count + np.arange(count)) % self.cap
        out = (self.idx[order], self.a[order], self.d[order])
        return out + (self.c[order],) if with_counts else out

    @property
    def last_idx(self) -> Optional[int]:
        return int(self.idx[(self.head - 1) % self.cap]) if self.n else None

    def range(self, i0: int, i1: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Samples with i0 <= idx < i1 that are still buffered (may contain holes)."""
        idx, a, d = self.latest()
        lo, hi = np.searchsorted(idx, [i0, i1])
        return idx[lo:hi], a[lo:hi], d[lo:hi]


# ---------------------------------------------------------------------------
# Spectrum
# ---------------------------------------------------------------------------
def amplitude_spectrum(x: np.ndarray, fs: float, nfft: int) -> Tuple[np.ndarray, np.ndarray]:
    """Welch-averaged single-sided amplitude spectrum (Hann, 50 % overlap).

    Scaled so a sine of amplitude A at a bin centre reads A (same scaling as
    the firmware's dsp::dominantFrequency). x may be (n,) or (n, k).
    """
    x = np.asarray(x, dtype=np.float64)
    if x.ndim == 1:
        x = x[:, None]
    n = x.shape[0]
    nfft = int(min(nfft, n))
    if nfft < 16:
        return np.zeros(0), np.zeros((0, x.shape[1]))
    w = np.hanning(nfft)
    step = nfft // 2
    starts = range(0, n - nfft + 1, step)
    acc = np.zeros((nfft // 2 + 1, x.shape[1]))
    count = 0
    for s in starts:
        seg = x[s:s + nfft]
        seg = (seg - seg.mean(axis=0)) * w[:, None]
        X = np.fft.rfft(seg, axis=0)
        acc += np.abs(X) ** 2
        count += 1
    amp = 2.0 * np.sqrt(acc / count) / w.sum()
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    return f, amp


@dataclass
class Peak:
    freq_hz: float = 0.0
    amp: float = 0.0
    noise_floor: float = 0.0
    valid: bool = False


def dominant_peak(f: np.ndarray, amp: np.ndarray, fmin: float, fmax: float) -> Peak:
    """Largest bin in [fmin, fmax], parabolic interpolation; valid if >= 12 dB above median."""
    if len(f) < 4 or fmax <= fmin:
        return Peak()
    df = f[1] - f[0]
    k0 = max(1, int(math.ceil(fmin / df)))
    k1 = min(len(f) - 2, int(math.floor(fmax / df)))
    if k1 <= k0 + 2:
        return Peak()
    band = amp[k0:k1 + 1]
    k = k0 + int(np.argmax(band))
    delta = 0.0
    if k0 < k < k1:
        a, b, c = amp[k - 1], amp[k], amp[k + 1]
        den = a - 2 * b + c
        if abs(den) > 1e-12:
            delta = 0.5 * (a - c) / den
    floor = float(np.median(band))
    return Peak((k + delta) * df, float(amp[k]), floor, bool(amp[k] > 0 and amp[k] >= 4 * floor))


# ---------------------------------------------------------------------------
# Sampling / bandwidth verification
# ---------------------------------------------------------------------------
@dataclass
class SamplingCheck:
    odr_nominal: float = 0.0
    fs_measured: float = 0.0
    deviation_pct: float = 0.0
    nyquist: float = 0.0
    dlpf_bw: float = 0.0
    valid_max: float = 0.0
    alias_risk: bool = False
    missing_pct: float = 0.0
    dup_ratio: float = 0.0
    spectral_ok: bool = False
    target_band_covered: bool = False
    verdict: str = "waiting for data"
    problems: List[str] = field(default_factory=list)

    @property
    def fs(self) -> float:
        return self.fs_measured if self.fs_measured > 0 else self.odr_nominal


def sampling_check(cfg: Optional[DeviceConfig], status: Optional[dict], received: int, missing: int) -> SamplingCheck:
    c = SamplingCheck()
    if cfg is None:
        return c
    c.odr_nominal = cfg.odr_hz
    c.dlpf_bw = float(cfg.get("bw_hz", 0))
    c.valid_max = float(cfg.get("valid_max_hz", 0))
    c.alias_risk = bool(int(cfg.get("alias_risk", 0)))
    c.nyquist = c.odr_nominal / 2
    if status:
        c.fs_measured = float(status.get("fs_meas", 0))
        c.dup_ratio = float(status.get("dup_ratio", 0))
    if c.fs_measured > 0:
        c.deviation_pct = 100.0 * (c.fs_measured - c.odr_nominal) / c.odr_nominal
        c.nyquist = c.fs_measured / 2
    total = received + missing
    c.missing_pct = 100.0 * missing / total if total else 0.0

    if c.fs_measured <= 0:
        c.problems.append("sample rate not yet measured (needs ~1 s of data)")
    elif abs(c.deviation_pct) > 3.0:
        c.problems.append(f"measured rate differs {c.deviation_pct:+.2f} % from nominal")
    if c.alias_risk or c.valid_max <= 0:
        c.problems.append("profile has no validated band (DLPF corner too close to Nyquist)")
    if c.dup_ratio > 0.2:
        c.problems.append("many duplicate samples - sensor may not be updating")
    if c.missing_pct > 1.0:
        c.problems.append(f"{c.missing_pct:.2f} % of samples missing from the stream")
    c.spectral_ok = not c.problems
    c.target_band_covered = c.spectral_ok and c.valid_max >= TARGET_BAND_HZ[1]
    if c.spectral_ok:
        c.verdict = (f"frequencies valid up to {c.valid_max:.2f} Hz "
                     f"(fs {c.fs:.2f} Hz, DLPF {c.dlpf_bw:.2f} Hz)")
    else:
        c.verdict = "frequencies NOT validated: " + "; ".join(c.problems)
    return c


# ---------------------------------------------------------------------------
# Impact / decay analysis
# ---------------------------------------------------------------------------
@dataclass
class DecayResult:
    ok: bool = False
    notes: List[str] = field(default_factory=list)
    compression_peak: float = float("nan")     # m/s^2, signed, primary axis
    rebound_peak: float = float("nan")         # m/s^2, signed (opposite sign)
    rebound_ratio: float = float("nan")        # |rebound| / |compression|
    comp_to_rebound_ms: float = float("nan")
    lobes: int = 0                             # half-cycles above the noise threshold
    damped_freq_hz: float = float("nan")
    decay_rate_per_s: float = float("nan")     # sigma in exp(-sigma t)
    damping_ratio: float = float("nan")        # zeta
    log_decrement: float = float("nan")        # per full cycle
    fit_r2: float = float("nan")
    settling_ms: float = float("nan")          # until |x| stays below settle_frac * peak
    freq_valid: bool = False
    method: str = ""
    # For plotting (seconds relative to the event peak, primary axis)
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    x_fit: np.ndarray = field(default_factory=lambda: np.zeros(0))   # signal the lobes came from
    lobe_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    lobe_a: np.ndarray = field(default_factory=lambda: np.zeros(0))


def _sign_with_deadband(x: np.ndarray, thr: float) -> np.ndarray:
    """Sign of x, but values inside +-thr take the previous sign (no noise chatter)."""
    s = np.where(x > thr, 1, np.where(x < -thr, -1, 0))
    nz = np.flatnonzero(s)
    if len(nz) == 0:
        return s
    # forward-fill zeros with the last non-zero sign
    fill_idx = np.maximum.accumulate(np.where(s != 0, np.arange(len(s)), 0))
    out = s[fill_idx]
    out[:nz[0]] = s[nz[0]]
    return out


def _zero_crossings(x: np.ndarray, lobe_idx: np.ndarray) -> np.ndarray:
    """Fractional sample positions of the sign change between consecutive lobes."""
    out = []
    for a, b in zip(lobe_idx[:-1], lobe_idx[1:]):
        seg = x[a:b + 1]
        ch = np.flatnonzero(np.sign(seg[1:]) != np.sign(seg[0]))
        if len(ch) == 0:
            continue
        k = a + int(ch[0])  # x[k] still has the first lobe's sign, x[k+1] does not
        x0, x1 = x[k], x[k + 1]
        out.append(k + (x0 / (x0 - x1) if x0 != x1 else 0.0))
    return np.array(out)


def _lobes(x: np.ndarray, floor: float) -> Tuple[np.ndarray, np.ndarray]:
    """Index and signed value of the largest |x| in each half-cycle above `floor`."""
    sg = _sign_with_deadband(x, floor)
    bounds = np.concatenate(([0], np.flatnonzero(np.diff(sg) != 0) + 1, [len(x)]))
    li, lv = [], []
    for a, b in zip(bounds[:-1], bounds[1:]):
        if b > a:
            k = a + int(np.argmax(np.abs(x[a:b])))
            if abs(x[k]) >= floor:
                li.append(k)
                lv.append(x[k])
    return np.array(li, dtype=int), np.array(lv, dtype=float)


def _ringdown(li: np.ndarray, lv: np.ndarray, main: int, n: int) -> List[int]:
    """Alternating, non-growing lobe sequence starting at `main`."""
    seq = [main]
    for j in range(main + 1, len(lv)):
        if np.sign(lv[j]) == np.sign(lv[seq[-1]]):
            break
        if abs(lv[j]) > 1.1 * abs(lv[seq[-1]]):
            break  # a free decay is monotonic; growth means a new disturbance
        seq.append(j)
    if len(seq) >= 2 and li[seq[-1]] >= n - 2:
        seq = seq[:-1]  # last lobe cut by the window end
    return seq


def lowpass_zero_phase(x: np.ndarray, fs: float, fc: float) -> np.ndarray:
    """Zero-phase low-pass (FFT domain, raised-cosine transition fc..1.5 fc).

    Zero phase matters: a causal filter would delay the lobes and shift their
    timing relative to the unfiltered peaks. The record is mirrored before the
    FFT to avoid wrap-around ringing at the ends.
    """
    n = len(x)
    ext = np.concatenate([x[::-1], x, x[::-1]])
    X = np.fft.rfft(ext)
    f = np.fft.rfftfreq(len(ext), 1.0 / fs)
    H = np.clip((1.5 * fc - f) / (0.5 * fc), 0.0, 1.0)
    H = 0.5 - 0.5 * np.cos(np.pi * H)
    return np.fft.irfft(X * H, len(ext))[n:2 * n]


def analyze_event(x: np.ndarray, fs: float, peak_i: int, noise_rms: float,
                  compression_sign: int = 1, settle_frac: float = 0.1,
                  valid_max_hz: float = 0.0) -> DecayResult:
    """Analyse one impact on the primary-axis dynamic acceleration x.

    x          samples from shortly before the event start to the end of the analysis window
    peak_i     index of the event peak within x (from the device E line)
    noise_rms  RMS of the quiet signal before the event (sets the lobe floor)

    Method: x is split into half-cycles at zero crossings (with a noise
    deadband); each half-cycle's largest |x| is a "lobe".

    * Compression = the largest lobe whose sign matches `compression_sign`;
      rebound = the next opposite-sign lobe; rebound ratio = |rebound|/|compression|.
      These use the unfiltered signal (true peak accelerations).
    * Ring-down: alternating, non-growing lobes from the largest one, while
      above max(5*noise, 3 % of peak). A coarse damped frequency from their
      zero crossings sets a zero-phase low-pass at 2.5*fd that strips impact
      transients and higher modes; the lobes are then re-extracted from the
      filtered signal. For a linear viscous damper the lobe amplitudes fall as
      exp(-sigma t), half a period apart:
          sigma from a least-squares fit of ln(A_k) vs t_k
          fd = 1 / (2 * mean zero-crossing spacing),  wd = 2 pi fd
          zeta = sigma / sqrt(sigma^2 + wd^2),  log decrement = sigma / fd
      A low fit R^2 means the decay is not exponential (e.g. friction, or
      nonlinear/asymmetric damping) and zeta should not be trusted.
    * Settling time: from the main lobe until |x| stays below settle_frac * peak.
    """
    r = DecayResult()
    n = len(x)
    if n < 8 or fs <= 0 or not (0 <= peak_i < n):
        r.notes.append("not enough samples")
        return r
    x = np.asarray(x, dtype=np.float64)
    r.t, r.x = (np.arange(n) - peak_i) / fs, x
    peak = float(np.max(np.abs(x[max(0, peak_i - 5):peak_i + 6])))
    # 5 sigma: Gaussian noise exceeds 3 sigma several times per second at 1 kHz.
    floor = max(5.0 * noise_rms, 0.03 * peak, 1e-6)

    li, lv = _lobes(x, floor)
    if len(li) == 0:
        r.notes.append("no excursion above the noise floor")
        return r
    main = int(np.argmax(np.abs(lv)))

    # Compression / rebound (unfiltered)
    same = [j for j in range(len(lv)) if np.sign(lv[j]) == compression_sign]
    if same:
        c = max(same, key=lambda j: abs(lv[j]))
        r.compression_peak = float(lv[c])
        nxt = [j for j in range(c + 1, len(lv)) if np.sign(lv[j]) == -compression_sign]
        if nxt:
            rb = nxt[0]
            r.rebound_peak = float(lv[rb])
            r.rebound_ratio = abs(lv[rb]) / abs(lv[c])
            r.comp_to_rebound_ms = (li[rb] - li[c]) * 1000.0 / fs
        else:
            r.notes.append("no rebound above the noise floor (strongly damped)")
    else:
        r.notes.append("no lobe with the compression sign - check primary axis / sign setting")

    # Settling (unfiltered): last sample above settle_frac * peak after the main lobe
    thr = settle_frac * abs(lv[main])
    after = np.flatnonzero(np.abs(x[li[main]:]) > thr)
    last = li[main] + (after[-1] if len(after) else 0)
    if last >= n - 1 - int(0.02 * fs):
        r.notes.append("did not settle within the analysis window")
    else:
        r.settling_ms = (last - li[main]) * 1000.0 / fs

    # Ring-down: coarse pass, then refine on the low-passed signal
    seq = _ringdown(li, lv, main, n)
    xs = x
    if len(seq) >= 3:
        zc = _zero_crossings(x, li[seq])
        sp = np.diff(zc) if len(zc) >= 2 else np.diff(li[seq])
        fd0 = fs / (2.0 * float(np.mean(sp)))
        if 2.5 * fd0 < 0.4 * fs:
            xf = lowpass_zero_phase(x, fs, 2.5 * fd0)
            lif, lvf = _lobes(xf, floor)
            if len(lif):
                # main lobe of the filtered signal: the one nearest the unfiltered main lobe
                mf = int(np.argmin(np.abs(lif - li[main])))
                seqf = _ringdown(lif, lvf, mf, n)
                if len(seqf) >= 3:
                    li, lv, seq, xs = lif, lvf, seqf, xf
                    r.method = f"lowpass {2.5 * fd0:.2f} Hz, "
    r.x_fit = xs
    r.lobes = len(seq)
    lt = li[seq] / fs
    la = np.abs(lv[seq])
    r.lobe_t = (li[seq] - peak_i) / fs
    r.lobe_a = lv[seq]

    if len(seq) >= 3:
        # Half-period from interpolated zero crossings: crossings are where the
        # signal is steepest, so their timing is much less noise-sensitive than
        # the flat lobe maxima. Lobe spacing is the fallback.
        zc = _zero_crossings(xs, li[seq]) / fs
        spacing = np.diff(zc) if len(zc) >= 2 else np.diff(lt)
        half = float(np.mean(spacing))
        fd = 1.0 / (2.0 * half)
        A = np.vstack([np.ones_like(lt), lt]).T
        coef, *_ = np.linalg.lstsq(A, np.log(la), rcond=None)
        sigma = -float(coef[1])
        ss_res = float(np.sum((np.log(la) - A @ coef) ** 2))
        ss_tot = float(np.sum((np.log(la) - np.log(la).mean()) ** 2))
        r.fit_r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
        wd = 2 * math.pi * fd
        r.damped_freq_hz = fd
        r.decay_rate_per_s = sigma
        if sigma > 0:
            r.damping_ratio = sigma / math.sqrt(sigma ** 2 + wd ** 2)
            r.log_decrement = sigma / fd
        else:
            r.notes.append("amplitude not decaying (resonance or sustained excitation)")
        if len(spacing) > 1 and np.std(spacing) > 0.25 * half:
            r.notes.append("irregular lobe spacing - not a single-mode ring-down")
        if r.fit_r2 < 0.9:
            r.notes.append(f"decay not exponential (R2={r.fit_r2:.2f}) - zeta approximate")
        r.method += f"{len(seq)}-lobe exponential fit"
    elif len(seq) == 2:
        # One overshoot: zeta from the ratio of successive half-cycle peaks,
        # ratio = exp(-pi*zeta/sqrt(1-zeta^2)).
        ratio = la[1] / la[0]
        lr = -math.log(ratio) if ratio > 0 else float("inf")
        r.damping_ratio = lr / math.sqrt(math.pi ** 2 + lr ** 2) if math.isfinite(lr) else 1.0
        half = lt[1] - lt[0]
        r.damped_freq_hz = 1.0 / (2.0 * half)
        r.log_decrement = 2 * lr
        r.decay_rate_per_s = lr / half
        r.method = "single-overshoot ratio"
        r.notes.append("only one overshoot above noise - zeta from overshoot ratio (approximate)")
    else:
        r.method = "no oscillation"
        r.notes.append("no overshoot above noise: heavily damped (zeta >~ 0.6) or weak excitation")

    if math.isfinite(r.damped_freq_hz):
        r.freq_valid = 0 < r.damped_freq_hz <= valid_max_hz
        if not r.freq_valid:
            r.notes.append(f"ring-down frequency {r.damped_freq_hz:.2f} Hz is outside the validated band")
    r.ok = True
    return r


def noise_rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=np.float64)
    return float(np.sqrt(np.mean((x - x.mean()) ** 2))) if len(x) > 4 else 0.0
