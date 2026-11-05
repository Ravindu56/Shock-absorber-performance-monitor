import math
import unittest

import numpy as np

from dashboard.analysis import (HostProcessor, SampleBuffer, amplitude_spectrum, analyze_event,
                                dominant_peak, lowpass_zero_phase, sampling_check)
from dashboard.protocol import DeviceConfig

FS = 1000.0


def ringdown(zeta, fn, amp=30.0, n=1500, pre=100, noise=0.05, seed=0):
    rng = np.random.default_rng(seed)
    wn = 2 * math.pi * fn
    wd = wn * math.sqrt(1 - zeta ** 2)
    t = np.arange(n) / FS
    x = np.zeros(pre + n)
    x[pre:] = amp * np.exp(-zeta * wn * t) * np.sin(wd * t)
    return x + rng.normal(0, noise, len(x)), wd / (2 * math.pi)


class DecayTests(unittest.TestCase):
    def test_damping_ratio_and_frequency(self):
        for zeta, fn in [(0.05, 12), (0.1, 12), (0.2, 25), (0.3, 8)]:
            x, fd = ringdown(zeta, fn)
            r = analyze_event(x, FS, int(np.argmax(np.abs(x))), 0.05, valid_max_hz=200)
            self.assertAlmostEqual(r.damping_ratio, zeta, delta=0.05 * zeta, msg=f"zeta {zeta}")
            self.assertAlmostEqual(r.damped_freq_hz, fd, delta=0.02 * fd, msg=f"fd {fd}")
            self.assertGreater(r.fit_r2, 0.98)
            self.assertTrue(r.freq_valid)

    def test_compression_rebound(self):
        x, _ = ringdown(0.2, 10)
        r = analyze_event(x, FS, int(np.argmax(np.abs(x))), 0.05, compression_sign=1)
        self.assertGreater(r.compression_peak, 0)
        self.assertLess(r.rebound_peak, 0)
        expected = math.exp(-math.pi * 0.2 / math.sqrt(1 - 0.04))  # successive half-cycle ratio
        self.assertAlmostEqual(r.rebound_ratio, expected, delta=0.05)
        # Inverted mounting: the same signal with the opposite sign convention
        r2 = analyze_event(-x, FS, int(np.argmax(np.abs(x))), 0.05, compression_sign=-1)
        self.assertAlmostEqual(r2.rebound_ratio, r.rebound_ratio, places=6)

    def test_overdamped_reports_no_oscillation(self):
        x, _ = ringdown(0.8, 15, seed=1)
        r = analyze_event(x, FS, int(np.argmax(np.abs(x))), 0.05)
        self.assertEqual(r.method, "no oscillation")
        self.assertTrue(math.isnan(r.damping_ratio))
        self.assertTrue(math.isfinite(r.settling_ms))

    def test_frequency_outside_validated_band_is_flagged(self):
        x, _ = ringdown(0.05, 150)
        r = analyze_event(x, FS, int(np.argmax(np.abs(x))), 0.05, valid_max_hz=90)
        self.assertFalse(r.freq_valid)

    def test_lowpass_is_zero_phase(self):
        t = np.arange(2000) / FS
        x = np.sin(2 * np.pi * 10 * t)
        y = lowpass_zero_phase(x + 0.5 * np.sin(2 * np.pi * 200 * t), FS, 40)
        self.assertLess(np.max(np.abs(y[200:-200] - x[200:-200])), 0.02)


class SpectrumTests(unittest.TestCase):
    def test_tone_amplitude_and_frequency(self):
        rng = np.random.default_rng(2)
        t = np.arange(8192) / FS
        x = 0.5 * np.sin(2 * np.pi * 123.4 * t) + rng.normal(0, 0.05, len(t))
        f, a = amplitude_spectrum(x, FS, 1024)
        p = dominant_peak(f, a[:, 0], 2, 200)
        self.assertTrue(p.valid)
        self.assertAlmostEqual(p.freq_hz, 123.4, delta=0.3)
        self.assertAlmostEqual(p.amp, 0.5, delta=0.5 * 0.16)  # Hann scalloping <= 1.42 dB

    def test_no_peak_outside_band(self):
        f, a = amplitude_spectrum(np.random.default_rng(3).normal(0, 1, 4096), FS, 1024)
        self.assertFalse(dominant_peak(f, a[:, 0], 2, 0).valid)


class SamplingCheckTests(unittest.TestCase):
    def cfg(self, **kw):
        v = dict(odr_hz=1000.0, bw_hz=218.1, valid_max_hz=200.0, alias_risk=0)
        v.update(kw)
        return DeviceConfig(v)

    def test_good(self):
        c = sampling_check(self.cfg(), {"fs_meas": 1004.0, "dup_ratio": 0.0}, 10000, 0)
        self.assertTrue(c.spectral_ok)
        self.assertFalse(c.target_band_covered)  # 500-2000 Hz is never covered by this sensor setup

    def test_rate_deviation(self):
        self.assertFalse(sampling_check(self.cfg(), {"fs_meas": 960.0}, 10000, 0).spectral_ok)

    def test_alias_profile(self):
        c = sampling_check(self.cfg(valid_max_hz=0.0, alias_risk=1), {"fs_meas": 1000.0}, 1000, 0)
        self.assertFalse(c.spectral_ok)

    def test_missing_samples(self):
        self.assertFalse(sampling_check(self.cfg(), {"fs_meas": 1000.0}, 9000, 1000).spectral_ok)


class HostProcessorTests(unittest.TestCase):
    def test_matches_firmware_baseline_filter(self):
        cfg = DeviceConfig(dict(gen=1, odr_hz=1000.0, lsb_per_g=2048.0, baseline_tau_s=0.5,
                                ofs_x=0.1, ofs_y=0, ofs_z=-0.2, scl_x=1.01, scl_y=1, scl_z=0.99))
        hp = HostProcessor()
        hp.set_config(cfg)
        rng = np.random.default_rng(4)
        counts = rng.integers(-3000, 3000, (200, 3)).astype(np.int16)
        _, a1, d1, _ = hp.process(0, counts[:50])
        _, a2, d2, _ = hp.process(50, counts[50:])
        d = np.vstack([d1, d2])
        # Reference: sample-by-sample recursion as in dsp::BaselineFilter
        a = (counts * (9.80665 / 2048.0) - cfg.offset) * cfg.scale
        alpha = 1 - math.exp(-1 / (1000 * 0.5))
        b = a[0].copy()
        ref = np.empty_like(a)
        for k in range(len(a)):
            b += alpha * (a[k] - b)
            ref[k] = a[k] - b
        np.testing.assert_allclose(d, ref, atol=1e-4)

    def test_gap_and_restart(self):
        hp = HostProcessor()
        hp.set_config(DeviceConfig(dict(gen=1, odr_hz=1000.0, lsb_per_g=2048.0)))
        c = np.zeros((50, 3), np.int16)
        hp.process(0, c)
        hp.process(80, c)
        self.assertEqual(hp.missing, 30)
        self.assertTrue(hp.process(0, c)[3])


class BufferTests(unittest.TestCase):
    def test_wraparound(self):
        b = SampleBuffer(100)
        for start in range(0, 250, 50):
            i = np.arange(start, start + 50)
            b.append(i, np.zeros((50, 3)), np.zeros((50, 3)), np.zeros((50, 3), np.int16))
        idx, _, _ = b.latest()
        np.testing.assert_array_equal(idx, np.arange(150, 250))
        idx, _, _ = b.range(180, 200)
        np.testing.assert_array_equal(idx, np.arange(180, 200))


if __name__ == "__main__":
    unittest.main()
