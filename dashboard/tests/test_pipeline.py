"""End-to-end tests: simulator -> protocol parser -> session -> recorder -> compare."""

import csv
import io
import math
import os
import tempfile
import unittest
from contextlib import redirect_stdout

import numpy as np

from dashboard import calibration, compare
from dashboard.protocol import Parser
from dashboard.recorder import read_summary
from dashboard.session import Session
from dashboard.simulator import DeviceSimulator


def run(sim, parser, session, seconds, dt=0.02):
    for _ in range(int(round(seconds / dt))):
        for line in sim.step(dt):
            msg = parser.parse(line)
            if msg:
                session.handle(msg)
        session.tick()


class ProtocolTests(unittest.TestCase):
    def test_all_simulator_lines_parse(self):
        sim, p = DeviceSimulator(seed=1), Parser()
        sim.hello()
        kinds = set()
        for _ in range(300):
            for line in sim.step(0.02):
                m = p.parse(line)
                kinds.add(m.kind)
        self.assertEqual(p.errors, 0, p.last_error)
        self.assertTrue({"HELLO", "H", "CFG", "S", "TS", "R", "E", "F"} <= kinds, kinds)

    def test_firmware_line_formats(self):
        # Lines exactly as printed by src/serial_protocol.cpp
        p = Parser()
        m = p.parse("E,3,1500,1504,1521,1500.123,17.1690,0.1,-0.2,17.0,21.00,2950.00,0")
        self.assertEqual(m.kind, "E")
        self.assertEqual(m.data["peak_idx"], 1504)
        m = p.parse("ACK,SET,ERR,hysteresis must be below threshold")
        self.assertFalse(m.data["ok"])
        m = p.parse("R,7,100,2,AQACAAMA//8AAAEA")
        self.assertEqual(m.data.counts.tolist(), [[1, 2, 3], [-1, 0, 1]])
        self.assertEqual(p.parse("R,7,100,3,AQACAAMA//8AAAEA").kind, "UNKNOWN")  # length mismatch
        self.assertEqual(p.parse("# booting").kind, "LOG")
        self.assertEqual(p.errors, 1)


class PipelineTests(unittest.TestCase):
    def test_events_decay_and_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim, p = DeviceSimulator(seed=3), Parser()
            s = Session(record_dir=tmp)
            s.s.analysis_timeout_s = 1e9  # deterministic: wait for the full window
            sim.hello()
            run(sim, p, s, 2.0)
            path = s.start_recording("unit test")
            run(sim, p, s, 40.0)
            s.stop_recording()

            self.assertAlmostEqual(s.fs, 1008.0, delta=0.5)        # simulated oscillator error is measured
            self.assertTrue(s.sampling().spectral_ok)
            res = [e.result for e in s.events if e.result and math.isfinite(e.result.damping_ratio)]
            self.assertGreaterEqual(len(res), 10)
            zeta = np.mean([r.damping_ratio for r in res])
            fd = np.mean([r.damped_freq_hz for r in res])
            self.assertAlmostEqual(zeta, 0.25, delta=0.025)          # simulator truth: zeta 0.25
            self.assertAlmostEqual(fd, 14 * math.sqrt(1 - 0.25 ** 2), delta=0.3)

            for f in ("samples.csv", "events.csv", "status.csv", "summary.csv"):
                self.assertTrue(os.path.isfile(os.path.join(path, f)), f)
            with open(os.path.join(path, "samples.csv")) as f:
                rows = list(csv.DictReader(f))
            self.assertGreater(len(rows), 39000)
            idx = np.array([int(r["idx"]) for r in rows])
            self.assertTrue(np.all(np.diff(idx) == 1))
            az = np.mean([float(r["az_ms2"]) for r in rows])
            self.assertAlmostEqual(az, 9.8, delta=0.6)               # uncalibrated sensor errors
            summ = read_summary(os.path.join(path, "summary.csv"))
            self.assertEqual(summ["label"], "unit test")
            self.assertGreaterEqual(int(summ["events"]), 10)
            self.assertIn("damping_ratio_mean", summ)

            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(compare.main([path, "-o", os.path.join(tmp, "cmp.csv")]), 0)
            self.assertIn("unit test", out.getvalue())

    def test_six_position_calibration_recovers_sensor_errors(self):
        sim, p = DeviceSimulator(seed=5), Parser()
        s = Session(record_dir=tempfile.gettempdir())
        caps = {}
        s.cal_listeners.append(lambda m: caps.__setitem__(pending[0], m))
        sim.hello()
        run(sim, p, s, 0.5)
        for pos in calibration.POSITIONS:
            pending = [pos]
            sim.command(f"SIM_ORIENT {pos}")
            run(sim, p, s, 0.3)
            sim.command("CAL 1000")
            run(sim, p, s, 1.3)
        self.assertEqual(set(caps), set(calibration.POSITIONS))
        sol = calibration.solve({k: calibration.CalCapture(
            np.array([m["mean_x"], m["mean_y"], m["mean_z"]]),
            np.array([m["std_x"], m["std_y"], m["std_z"]]), m["samples"]) for k, m in caps.items()})
        self.assertEqual(sol.warnings, [])
        np.testing.assert_allclose(sol.offset, sim.true_ofs, atol=0.01)
        np.testing.assert_allclose(sol.scale, 1 / sim.true_scl, atol=0.002)
        # Apply and verify |g| in the current orientation
        for c in sol.commands():
            sim.command(c)
        sim.command("CAL 1000")
        run(sim, p, s, 1.3)
        rep = calibration.stationary_report(s.last_cal, s.cfg.offset, s.cfg.scale)
        self.assertLess(abs(rep.mag_error_pct), 0.2)

    def test_tare_zeroes_z_without_false_impact(self):
        sim, p = DeviceSimulator(seed=7), Parser()
        s = Session(record_dir=tempfile.gettempdir())
        sim.hello()
        sim.command("SIM_BUMPS 0")
        run(sim, p, s, 1.0)
        acks = []
        s.cal_listeners.clear()
        sim.command("TARE")
        for _ in range(int(2.0 / 0.02)):
            for line in sim.step(0.02):
                m = p.parse(line)
                if m:
                    s.handle(m)
                    if m.kind == "ACK" and m.data["cmd"] == "TARE":
                        acks.append(m.data)
        self.assertTrue(acks[-1]["ok"], acks)
        self.assertAlmostEqual(s.cfg.tare[2], 9.8, delta=0.6)   # gravity (+ sensor errors) captured
        run(sim, p, s, 1.0)
        _, a, _ = s.buffer.latest(500)
        self.assertLess(abs(float(np.mean(a[:, 2]))), 0.02)       # Z now reads ~0 m/s^2
        self.assertEqual(len(s.events), 0)                         # the step did not count as an impact

        sim.command("SIM_BUMPS 1")                                 # vibrating: zeroing must be refused
        sim._next_bump = sim.idx / sim.fs_true + 0.6
        sim.command("TARE")
        acks.clear()
        for _ in range(int(2.0 / 0.02)):
            for line in sim.step(0.02):
                m = p.parse(line)
                if m and m.kind == "ACK" and m.data["cmd"] == "TARE":
                    acks.append(m.data)
        self.assertFalse(acks[-1]["ok"], acks)

    def test_set_validation(self):
        sim, p = DeviceSimulator(seed=1), Parser()
        sim.command("SET HYSTERESIS 50")
        ack = p.parse(sim.step(0.0)[-1])
        self.assertEqual(ack.kind, "ACK")
        self.assertFalse(ack.data["ok"])


if __name__ == "__main__":
    unittest.main()
