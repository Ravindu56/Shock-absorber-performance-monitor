"""PyQtGraph desktop dashboard for the ESP32/MPU6500 shock absorber monitor.

Run:  python -m dashboard                 (pick the serial port in the toolbar)
      python -m dashboard --port /dev/cu.usbserial-0001
      python -m dashboard --simulate      (no hardware: built-in device simulator)
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Optional

import numpy as np
import pyqtgraph as pg
import serial
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from . import calibration
from .analysis import TARGET_BAND_HZ, amplitude_spectrum, dominant_peak
from .links import Link, SerialLink, SimulatedLink, describe_ports, find_esp32_ports
from .protocol import FLAG_PROBLEMS, FLAGS, G, RANGES_G, SENSOR_PROFILES
from .session import EventRecord, Session, Settings

SIM_PORT = "Simulator (no hardware)"
AXIS_COLORS = ["#ff6b6b", "#51cf66", "#4dabf7"]
MONO = QtGui.QFont("Menlo" if sys.platform == "darwin" else "Consolas", 11)

Qt = QtCore.Qt

# Device parameters editable in the Settings panel:
#   (SET key, CFG key, label, kind, min, max, decimals/step, tooltip)
DEVICE_PARAMS = [
    ("PROFILE", "profile", "Sensor profile", "combo", SENSOR_PROFILES, None, None,
     "ODR / DLPF bandwidth combination (see README 'Bandwidth and frequency limits')."),
    ("RANGE", "range_g", "Range (+-g)", "combo", [str(r) for r in RANGES_G], None, None,
     "Full-scale range. Use the smallest range that never clips (watch the CLIP flag)."),
    ("AXIS", "primary_axis", "Monitored axis", "combo", ["X", "Y", "Z"], None, None,
     "Sensor axis aligned with the shock absorber stroke. Only this axis is monitored (default Z)."),
    ("THRESHOLD", "threshold", "Impact threshold", "float", 0.05, 1000.0, 2,
     "Trigger level on |Z dynamic acceleration| [m/s^2]."),
    ("HYSTERESIS", "hysteresis", "Hysteresis", "float", 0.0, 1000.0, 2,
     "Event ends below threshold - hysteresis [m/s^2]."),
    ("PERSIST", "persist", "Persist (samples)", "int", 1, 100, 1,
     "Consecutive samples above threshold needed to trigger (rejects single spikes)."),
    ("WINDOW_MS", "event_window_ms", "Max event (ms)", "int", 1, 10000, 10,
     "Maximum event length; longer events are flagged as truncated."),
    ("RELEASE_MS", "release_hold_ms", "Release hold (ms)", "int", 0, 1000, 5,
     "Time below the release level needed to end an event."),
    ("REFRACTORY_MS", "refractory_ms", "Refractory (ms)", "int", 0, 60000, 10,
     "Dead time after an event before a new one may start."),
    ("TAU", "baseline_tau_s", "Baseline tau (s)", "float", 0.05, 30.0, 2,
     "Gravity/baseline removal time constant; high-pass corner = 1/(2 pi tau)."),
    ("RMS_MS", "rms_window_ms", "RMS window (ms)", "int", 10, 10000, 10,
     "Block length of the RMS / P2P vibration window."),
    ("REPORT_HZ", "report_hz", "Status rate (Hz)", "int", 1, 100, 1, "Status (S) lines per second."),
    ("FFT_N", "fft_n", "Device FFT N", "combo", ["256", "512", "1024", "2048"], None, None,
     "On-device FFT length; resolution = fs / N."),
    ("FFT_MS", "fft_interval_ms", "Device FFT every (ms)", "int", 200, 60000, 100, ""),
    ("FMIN", "fft_fmin_hz", "FFT min freq (Hz)", "float", 0.0, 500.0, 2,
     "Ignore spectral peaks below this frequency."),
    ("STREAM", "stream", "Stream raw samples", "check", None, None, None,
     "Full-rate raw data for graphs, decay analysis and recording."),
]


def fmt(v: float, nd: int = 2) -> str:
    return "—" if v is None or not math.isfinite(v) else f"{v:.{nd}f}"


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__()
        self.args = args
        self.setWindowTitle("Shock Absorber Monitor — ESP32 + MPU6500")
        icon = resource_path("packaging", "icon.png")
        if os.path.isfile(icon):
            self.setWindowIcon(QtGui.QIcon(icon))
        self.resize(1500, 950)
        self.session = Session(Settings(buffer_s=args.buffer_s), record_dir=args.record_dir)
        self.session.cal_listeners.append(self._on_cal_result)
        self.link: Optional[Link] = None
        self.connected_at = 0.0
        self.last_hello_req = 0.0
        self.units_g = False
        self.paused = False
        self.selected_event: Optional[EventRecord] = None
        self.cal_pending: Optional[str] = None     # 6-position capture in progress ("+X" ...)
        self.cal_started = 0.0
        self.cal_caps: Dict[str, calibration.CalCapture] = {}
        self.cal_solution: Optional[calibration.CalSolution] = None
        self._last_flush = time.monotonic()
        self._rate_t, self._rate_lines, self.line_rate = time.monotonic(), 0, 0.0
        self._settings_dirty = False

        pg.setConfigOptions(antialias=False, foreground="#ddd", background="#1e1f22")
        self._build_toolbar()
        self._build_central()
        self._build_side()
        self.status_lbl = QtWidgets.QLabel("disconnected")
        self.statusBar().addWidget(self.status_lbl, 1)
        self.rec_lbl = QtWidgets.QLabel("")
        self.statusBar().addPermanentWidget(self.rec_lbl)

        self.data_timer = QtCore.QTimer(self, timeout=self._pump, interval=20)
        self.plot_timer = QtCore.QTimer(self, timeout=self._update_plots, interval=40)
        self.slow_timer = QtCore.QTimer(self, timeout=self._update_slow, interval=250)
        self.spec_timer = QtCore.QTimer(self, timeout=self._update_spectrum, interval=500)
        for t in (self.data_timer, self.plot_timer, self.slow_timer, self.spec_timer):
            t.start()

        # Plug and play: while disconnected, scan for an ESP32 USB adapter every
        # second and connect to it; reconnect after the cable is unplugged/replugged.
        self.auto_timer = QtCore.QTimer(self, timeout=self._auto_connect, interval=1000)
        self.auto_timer.start()
        if args.simulate:
            self.auto_chk.setChecked(False)
            self._select_port(SIM_PORT)
            self._connect()
        elif args.port:
            self.auto_chk.setChecked(False)
            self._select_port(args.port)
            self._connect()

    # ------------------------------------------------------------------ UI
    def _build_toolbar(self) -> None:
        tb = self.addToolBar("main")
        tb.setMovable(False)
        self.port_combo = QtWidgets.QComboBox(minimumWidth=230)
        self._refresh_ports()
        tb.addWidget(QtWidgets.QLabel(" Port "))
        tb.addWidget(self.port_combo)
        tb.addAction("↻", self._refresh_ports).setToolTip("Rescan serial ports")
        self.auto_chk = QtWidgets.QCheckBox("Auto-connect", checked=True,
                                            toolTip="Connect automatically when an ESP32 board is plugged in")
        tb.addWidget(self.auto_chk)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(lambda *_: self._toggle_connect())
        tb.addWidget(self.connect_btn)
        tb.addSeparator()
        for text, cmd, tip in [("Start", "START", "Start acquisition"),
                               ("Stop", "STOP", "Stop acquisition"),
                               ("Reset stats", "RESET", "Clear test statistics and impact count")]:
            b = QtWidgets.QPushButton(text, toolTip=tip)
            b.clicked.connect(lambda *_, c=cmd: self._send(c))
            tb.addWidget(b)
        tb.addSeparator()
        b = QtWidgets.QPushButton("Zero Z", toolTip="Same as the GPIO4 button: with the sensor still, make the "
                                  "Z acceleration read 0 (gravity removed). Saved to flash.")
        b.clicked.connect(lambda *_: self._send("TARE"))
        tb.addWidget(b)
        b = QtWidgets.QPushButton("Clear zero", toolTip="Remove the zero offset (Z shows gravity again)")
        b.clicked.connect(lambda *_: self._clear_tare())
        tb.addWidget(b)
        tb.addSeparator()
        tb.addWidget(QtWidgets.QLabel(" Units "))
        self.units_combo = QtWidgets.QComboBox()
        self.units_combo.addItems(["m/s²", "g"])
        self.units_combo.currentIndexChanged.connect(self._units_changed)
        tb.addWidget(self.units_combo)
        self.pause_chk = QtWidgets.QCheckBox("Freeze plots")
        self.pause_chk.toggled.connect(lambda v: setattr(self, "paused", v))
        tb.addWidget(self.pause_chk)
        tb.addSeparator()
        tb.addWidget(QtWidgets.QLabel(" Test label "))
        self.label_edit = QtWidgets.QLineEdit("prototype_A", maximumWidth=160)
        tb.addWidget(self.label_edit)
        self.reset_on_rec = QtWidgets.QCheckBox("reset stats", checked=True,
                                                toolTip="Send RESET when recording starts")
        tb.addWidget(self.reset_on_rec)
        self.rec_btn = QtWidgets.QPushButton("● Record")
        self.rec_btn.setCheckable(True)
        self.rec_btn.toggled.connect(self._toggle_record)
        tb.addWidget(self.rec_btn)
        b = QtWidgets.QPushButton("Export snapshot",
                                  toolTip="Write the in-memory history and events to CSV now")
        b.clicked.connect(lambda *_: self._export_snapshot())
        tb.addWidget(b)

    def _build_central(self) -> None:
        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        # --- Live tab
        live = pg.GraphicsLayoutWidget()
        # Only the primary (stroke) axis is shown - Z unless AXIS is changed.
        self.p_acc = live.addPlot(row=0, col=0, title="Z acceleration (calibrated, incl. gravity)")
        self.p_dyn = live.addPlot(row=1, col=0, title="Z vibration (dynamic acceleration, gravity removed)")
        for p in (self.p_acc, self.p_dyn):
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setDownsampling(auto=True, mode="peak")
            p.setClipToView(True)
            p.setLabel("bottom", "time", units="s")
        self.p_dyn.setXLink(self.p_acc)
        self.c_acc = self.p_acc.plot(pen=pg.mkPen(AXIS_COLORS[2], width=1))
        self.c_dyn = self.p_dyn.plot(pen=pg.mkPen("#ffd43b", width=1))
        thr_pen = pg.mkPen("#ff6b6b", style=Qt.PenStyle.DashLine)
        self.thr_line = pg.InfiniteLine(angle=0, pen=thr_pen, label="+threshold",
                                        labelOpts={"position": 0.05, "color": "#ff6b6b"})
        self.thr_line_n = pg.InfiniteLine(angle=0, pen=thr_pen, label="-threshold",
                                          labelOpts={"position": 0.05, "color": "#ff6b6b"})
        self.p_dyn.addItem(self.thr_line)
        self.p_dyn.addItem(self.thr_line_n)
        self.event_lines: List[pg.InfiniteLine] = []
        for _ in range(30):
            ln = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ff922b", width=1))
            ln.hide()
            self.p_dyn.addItem(ln)
            self.event_lines.append(ln)
        self.tabs.addTab(live, "Live")

        # --- Spectrum tab
        spec = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(spec)
        self.spec_info = QtWidgets.QLabel(wordWrap=True)
        self.spec_info.setFont(MONO)
        v.addWidget(self.spec_info)
        self.p_spec = pg.PlotWidget(title="Z vibration amplitude spectrum (Welch, Hann)")
        self.p_spec.setLogMode(x=False, y=True)
        self.p_spec.showGrid(x=True, y=True, alpha=0.25)
        self.p_spec.setLabel("bottom", "frequency", units="Hz")
        self.p_spec.setLabel("left", "amplitude [m/s²]")
        self.c_spec = self.p_spec.plot(pen=pg.mkPen(AXIS_COLORS[2], width=1))
        self.invalid_region = pg.LinearRegionItem(movable=False, brush=pg.mkBrush(120, 120, 120, 70),
                                                  pen=pg.mkPen(None))
        self.p_spec.addItem(self.invalid_region)
        self.invalid_lbl = pg.TextItem("not validated", color="#aaa", anchor=(0, 0))
        self.p_spec.addItem(self.invalid_lbl)
        self.dlpf_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ff922b", style=Qt.PenStyle.DashLine),
                                         label="DLPF -3 dB", labelOpts={"position": 0.9, "color": "#ff922b"})
        self.p_spec.addItem(self.dlpf_line)
        self.peak_marker = pg.ScatterPlotItem(size=12, symbol="t", brush=pg.mkBrush("#ffd43b"))
        self.p_spec.addItem(self.peak_marker)
        v.addWidget(self.p_spec, 1)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Analysis length (s)"))
        self.spec_len = QtWidgets.QDoubleSpinBox(minimum=0.5, maximum=30, value=4, singleStep=0.5)
        row.addWidget(self.spec_len)
        row.addWidget(QtWidgets.QLabel("Segment N"))
        self.spec_n = QtWidgets.QComboBox()
        self.spec_n.addItems(["256", "512", "1024", "2048", "4096"])
        self.spec_n.setCurrentText("1024")
        row.addWidget(self.spec_n)
        row.addStretch(1)
        v.addLayout(row)
        self.tabs.addTab(spec, "Spectrum")

        # --- Impact analysis tab
        imp = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        top = QtWidgets.QWidget()
        tv = QtWidgets.QVBoxLayout(top)
        tv.setContentsMargins(0, 0, 0, 0)
        hdr = QtWidgets.QHBoxLayout()
        self.follow_chk = QtWidgets.QCheckBox("Show latest event", checked=True)
        hdr.addWidget(self.follow_chk)
        hdr.addStretch(1)
        tv.addLayout(hdr)
        self.p_evt = pg.PlotWidget(title="Selected event — Z acceleration relative to the pre-impact level")
        self.p_evt.showGrid(x=True, y=True, alpha=0.25)
        self.p_evt.setLabel("bottom", "time from peak", units="s")
        self.p_evt.addLegend(offset=(-10, 5))
        self.c_evt = self.p_evt.plot(pen=pg.mkPen("#4dabf7", width=1), name="measured")
        self.c_evt_f = self.p_evt.plot(pen=pg.mkPen("#e599f7", width=2), name="low-passed (fit input)")
        self.c_env_p = self.p_evt.plot(pen=pg.mkPen("#ffd43b", style=Qt.PenStyle.DashLine))
        self.c_env_n = self.p_evt.plot(pen=pg.mkPen("#ffd43b", style=Qt.PenStyle.DashLine))
        self.s_lobes = pg.ScatterPlotItem(size=8, brush=pg.mkBrush("#ff922b"), name="ring-down lobes")
        self.p_evt.addItem(self.s_lobes)
        self.evt_info = QtWidgets.QLabel(wordWrap=True)
        self.evt_info.setFont(MONO)
        tv.addWidget(self.p_evt, 1)
        tv.addWidget(self.evt_info)
        imp.addWidget(top)
        self.evt_cols = ["#", "t (s)", "peak |dz|", "peak g", "dur ms", "compr.", "rebound", "reb/comp",
                         "fd Hz", "ζ", "settle ms", "R²", "notes"]
        self.evt_table = QtWidgets.QTableWidget(0, len(self.evt_cols))
        self.evt_table.setHorizontalHeaderLabels(self.evt_cols)
        self.evt_table.horizontalHeader().setStretchLastSection(True)
        self.evt_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.evt_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.evt_table.itemSelectionChanged.connect(self._event_selected)
        imp.addWidget(self.evt_table)
        imp.setSizes([500, 300])
        self.tabs.addTab(imp, "Impact analysis")

        # --- Calibration tab
        self.tabs.addTab(self._build_cal_tab(), "Calibration")

        # --- Log tab
        self.log_view = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log_view.setMaximumBlockCount(3000)
        self.log_view.setFont(MONO)
        logw = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(logw)
        lv.addWidget(self.log_view)
        row = QtWidgets.QHBoxLayout()
        self.cmd_edit = QtWidgets.QLineEdit(placeholderText="raw command, e.g. SET THRESHOLD 15 / GETCFG / PING")
        self.cmd_edit.returnPressed.connect(self._send_raw)
        row.addWidget(self.cmd_edit)
        lv.addLayout(row)
        self.tabs.addTab(logw, "Log")

    def _build_cal_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        intro = QtWidgets.QLabel(
            "<b>Stationary check</b>: place the sensor still and press the button. It reports the "
            "gravity magnitude (should be 9.807 m/s² after calibration), per-axis noise and tilt.<br>"
            "<b>6-position calibration</b>: rest the sensor so each axis in turn points straight up (+) "
            "and straight down (−), keep it still, and capture each position (3 s). Then compute, "
            "apply and save to flash. Offsets/scales are applied in the firmware.")
        intro.setWordWrap(True)
        v.addWidget(intro)
        row = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("Run stationary check (3 s)")
        b.clicked.connect(lambda *_: self._cal_capture(None))
        row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)
        self.cal_station_lbl = QtWidgets.QLabel("—", wordWrap=True)
        self.cal_station_lbl.setFont(MONO)
        v.addWidget(self.cal_station_lbl)

        grid = QtWidgets.QGridLayout()
        self.cal_pos_lbl: Dict[str, QtWidgets.QLabel] = {}
        for i, pos in enumerate(calibration.POSITIONS):
            b = QtWidgets.QPushButton(f"Capture {pos} axis up")
            b.clicked.connect(lambda *_, p=pos: self._cal_capture(p))
            grid.addWidget(b, i, 0)
            lbl = QtWidgets.QLabel("not captured")
            lbl.setFont(MONO)
            grid.addWidget(lbl, i, 1)
            self.cal_pos_lbl[pos] = lbl
        grid.setColumnStretch(1, 1)
        v.addLayout(grid)
        row = QtWidgets.QHBoxLayout()
        for text, fn in [("Compute", self._cal_compute), ("Apply to device", self._cal_apply),
                         ("Save to flash", lambda: self._send("SAVE")),
                         ("Clear calibration", self._cal_clear), ("Reset captures", self._cal_reset)]:
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda *_, f=fn: f())
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)
        self.cal_result_lbl = QtWidgets.QLabel("", wordWrap=True)
        self.cal_result_lbl.setFont(MONO)
        v.addWidget(self.cal_result_lbl)
        self.cal_current_lbl = QtWidgets.QLabel("", wordWrap=True)
        self.cal_current_lbl.setFont(MONO)
        v.addWidget(self.cal_current_lbl)
        v.addStretch(1)
        return w

    def _build_side(self) -> None:
        dock = QtWidgets.QDockWidget("Measurements and settings", self)
        dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        inner = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(inner)

        def group(title: str, rows: List[str]) -> Dict[str, QtWidgets.QLabel]:
            g = QtWidgets.QGroupBox(title)
            gl = QtWidgets.QGridLayout(g)
            gl.setVerticalSpacing(2)
            out = {}
            for i, name in enumerate(rows):
                gl.addWidget(QtWidgets.QLabel(name), i, 0)
                val = QtWidgets.QLabel("—")
                val.setFont(MONO)
                val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                val.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
                gl.addWidget(val, i, 1)
                out[name] = val
            v.addWidget(g)
            return out

        self.v_acc = group("Z acceleration (latest)", ["m/s²", "g", "Zero offset"])
        self.v_vib = group("Z vibration (device, per window)", ["RMS", "Peak-to-peak", "Peak (interval)",
                                                                "Dominant freq"])
        self.v_test = group("Z test since reset (device)", ["Max |dz|", "RMS", "Peak-to-peak", "Impacts",
                                                            "Clipped samples"])
        self.v_samp = group("Sampling and bandwidth verification", [
            "Nominal ODR", "Measured fs", "Deviation", "DLPF bandwidth", "Nyquist", "Validated band",
            "Stream missing", "Duplicates", "FIFO ovf / I2C err", "Spectral data"])
        self.samp_verdict = QtWidgets.QLabel(wordWrap=True)
        v.addWidget(self.samp_verdict)
        self.target_lbl = QtWidgets.QLabel(wordWrap=True)
        v.addWidget(self.target_lbl)
        self.flags_lbl = QtWidgets.QLabel(wordWrap=True)
        v.addWidget(self.flags_lbl)

        # Device settings
        g = QtWidgets.QGroupBox("Device settings")
        form = QtWidgets.QFormLayout(g)
        self.param_w: Dict[str, QtWidgets.QWidget] = {}
        for key, ckey, label, kind, lo, hi, dec, tip in DEVICE_PARAMS:
            if kind == "combo":
                w = QtWidgets.QComboBox()
                w.addItems(lo)
                w.currentIndexChanged.connect(self._mark_dirty)
            elif kind == "float":
                w = QtWidgets.QDoubleSpinBox(minimum=lo, maximum=hi, decimals=dec, singleStep=10 ** -dec * 10)
                w.valueChanged.connect(self._mark_dirty)
            elif kind == "int":
                w = QtWidgets.QSpinBox(minimum=lo, maximum=hi, singleStep=dec)
                w.valueChanged.connect(self._mark_dirty)
            else:
                w = QtWidgets.QCheckBox()
                w.toggled.connect(self._mark_dirty)
            w.setToolTip(tip)
            self.param_w[key] = w
            form.addRow(label, w)
        self.thr_g_lbl = QtWidgets.QLabel()
        form.addRow("", self.thr_g_lbl)
        self.param_w["THRESHOLD"].valueChanged.connect(
            lambda val: self.thr_g_lbl.setText(f"threshold = {val / G:.2f} g"))
        row = QtWidgets.QHBoxLayout()
        for text, fn in [("Apply", self._apply_settings), ("Read", lambda: self._send("GETCFG")),
                         ("Save to flash", lambda: self._send("SAVE")),
                         ("Defaults", lambda: self._send("DEFAULTS"))]:
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda *_, f=fn: f())
            row.addWidget(b)
        form.addRow(row)
        self.dirty_lbl = QtWidgets.QLabel("")
        form.addRow(self.dirty_lbl)
        v.addWidget(g)

        # Host analysis settings
        g = QtWidgets.QGroupBox("Analysis settings (dashboard)")
        form = QtWidgets.QFormLayout(g)
        self.h_sign = QtWidgets.QComboBox()
        self.h_sign.addItems(["+ (positive = compression)", "− (negative = compression)"])
        self.h_sign.setToolTip("Sign of the primary-axis acceleration during compression for this mounting.")
        self.h_sign.currentIndexChanged.connect(
            lambda i: setattr(self.session.s, "compression_sign", 1 if i == 0 else -1))
        form.addRow("Compression sign", self.h_sign)
        self.h_win = QtWidgets.QSpinBox(minimum=100, maximum=10000, value=int(self.session.s.analysis_ms),
                                        singleStep=100, suffix=" ms")
        self.h_win.valueChanged.connect(lambda val: setattr(self.session.s, "analysis_ms", float(val)))
        form.addRow("Decay window", self.h_win)
        self.h_settle = QtWidgets.QSpinBox(minimum=1, maximum=50, value=int(self.session.s.settle_frac * 100),
                                           suffix=" % of peak")
        self.h_settle.valueChanged.connect(lambda val: setattr(self.session.s, "settle_frac", val / 100.0))
        form.addRow("Settling band", self.h_settle)
        self.h_plot = QtWidgets.QDoubleSpinBox(minimum=0.5, maximum=60, value=5, singleStep=1, suffix=" s")
        form.addRow("Plot window", self.h_plot)
        v.addWidget(g)
        v.addStretch(1)

        scroll.setWidget(inner)
        scroll.setMinimumWidth(400)
        dock.setWidget(scroll)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)

    # ------------------------------------------------------------ connection
    def _refresh_ports(self) -> None:
        cur = self.port_combo.currentData()
        self.port_combo.clear()
        for dev, label in describe_ports():
            self.port_combo.addItem(label, dev)
        self.port_combo.addItem(SIM_PORT, SIM_PORT)
        if cur:
            self._select_port(cur)

    def _select_port(self, dev: str) -> None:
        i = self.port_combo.findData(dev)
        if i < 0:
            self.port_combo.addItem(dev, dev)
            i = self.port_combo.findData(dev)
        self.port_combo.setCurrentIndex(i)

    def _toggle_connect(self) -> None:
        if self.link:
            self._disconnect(manual=True)
        else:
            self._connect()

    def _auto_connect(self) -> None:
        if self.link or not self.auto_chk.isChecked():
            return
        ports = find_esp32_ports()
        known = {self.port_combo.itemData(i) for i in range(self.port_combo.count())}
        if set(ports) - known:
            self._refresh_ports()
        if ports:
            self._select_port(ports[0])
            self._connect(quiet=True)

    def _connect(self, quiet: bool = False) -> None:
        port = self.port_combo.currentData() or self.port_combo.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self, "No port", "No serial port selected. Plug in the ESP32 and press ↻.")
            return
        try:
            link = SimulatedLink() if port == SIM_PORT else SerialLink(port, self.args.baud)
            link.start()
        except (serial.SerialException, OSError) as exc:
            if quiet:  # auto-connect: retry silently, the port may still be initialising
                self.statusBar().showMessage(f"Waiting for {port}: {exc}", 3000)
                return
            QtWidgets.QMessageBox.critical(self, "Connection failed", f"Could not open {port}:\n{exc}\n\n"
                                           "Is another program (e.g. the PlatformIO monitor) using the port?")
            return
        self.link = link
        self.connected_at = time.monotonic()
        self.last_hello_req = self.connected_at
        self.connect_btn.setText("Disconnect")
        self.port_combo.setEnabled(False)
        self.session._log(f"connected to {port}")
        if isinstance(link, SerialLink):
            # Opening the port toggles DTR/RTS, which resets most ESP32 DevKits; the
            # firmware then sends HELLO by itself. Ask again in case it did not reset.
            QtCore.QTimer.singleShot(1500, lambda: self._send("HELLO"))

    def _disconnect(self, manual: bool = False) -> None:
        if manual:
            self.auto_chk.setChecked(False)  # don't reconnect right after the user disconnected
        if self.rec_btn.isChecked():
            self.rec_btn.setChecked(False)
        if self.link:
            self.link.stop()
            self.session._log(f"disconnected from {self.link.name}")
        self.link = None
        self.connect_btn.setText("Connect")
        self.port_combo.setEnabled(True)

    def _send(self, cmd: str) -> None:
        if not self.link:
            self.session._log(f"not connected - '{cmd}' not sent")
            return
        self.link.send(cmd)
        self.session._log(f"> {cmd}")

    def _send_raw(self) -> None:
        cmd = self.cmd_edit.text().strip()
        if cmd:
            self._send(cmd)
            self.cmd_edit.clear()

    # ----------------------------------------------------------- data pump
    def _pump(self) -> None:
        if not self.link:
            return
        handled = 0
        cfg_before = self.session.cfg
        deadline = time.monotonic() + 0.015   # keep the GUI responsive under bursts
        while time.monotonic() < deadline:
            try:
                msg = self.link.rx.get_nowait()
            except Exception:
                break
            self.session.handle(msg)
            handled += 1
        self._rate_lines += handled
        for rec in self.session.tick():
            self._add_event_row(rec)
        if self.session.cfg is not cfg_before and self.session.cfg is not None:
            self._load_settings(self.session.cfg)
        if self.session.new_log:
            for line in self.session.new_log:
                self.log_view.appendPlainText(line)
            self.session.new_log.clear()
        now = time.monotonic()
        if now - self._last_flush > 1.0:
            self._last_flush = now
            self.session.recorder.flush()
        if not self.link.alive:
            self.session._log("link thread stopped")
            self._disconnect()

    # ---------------------------------------------------------------- plots
    def _units_changed(self, i: int) -> None:
        self.units_g = i == 1
        u = "g" if self.units_g else "m/s²"
        for p in (self.p_acc, self.p_dyn):
            p.setLabel("left", u)

    @property
    def uscale(self) -> float:
        return 1.0 / G if self.units_g else 1.0

    def _update_plots(self) -> None:
        if self.paused or self.session.cfg is None:
            return
        fs = self.session.fs
        n = int(self.h_plot.value() * fs)
        idx, a, d = self.session.buffer.latest(n)
        if len(idx) < 2:
            return
        t = (idx - idx[-1]) / fs
        u = self.uscale
        ax = self.session.cfg.primary_axis
        name = "XYZ"[ax]
        self.c_acc.setData(t, a[:, ax] * u)
        self.c_dyn.setData(t, d[:, ax] * u)
        zeroed = self.session.cfg.tare[ax] != 0
        self.p_acc.setTitle(f"{name} acceleration (calibrated, {'zeroed - gravity removed' if zeroed else 'incl. gravity'})")
        self.p_dyn.setTitle(f"{name} vibration (dynamic acceleration, gravity removed)")
        thr = float(self.session.cfg.get("threshold", 0)) * u
        self.thr_line.setValue(thr)
        self.thr_line_n.setValue(-thr)
        evs = [e for e in self.session.events[-len(self.event_lines):]]
        for ln, e in zip(self.event_lines, evs + [None] * (len(self.event_lines) - len(evs))):
            if e is None or e.ev["start_idx"] < idx[0]:
                ln.hide()
            else:
                ln.setValue((e.ev["start_idx"] - idx[-1]) / fs)
                ln.show()

    def _update_spectrum(self) -> None:
        if self.paused or self.session.cfg is None or self.tabs.currentIndex() != 1:
            return
        sc = self.session.sampling()
        fs = sc.fs
        idx, _, d = self.session.buffer.latest(int(self.spec_len.value() * fs))
        nseg = int(self.spec_n.currentText())
        if len(idx) < nseg or np.any(np.diff(idx) != 1):
            self.spec_info.setText(f"Waiting for {nseg} contiguous samples...")
            return
        ax = self.session.cfg.primary_axis
        f, amp = amplitude_spectrum(d[:, ax], fs, nseg)
        amp = np.maximum(amp, 1e-6)
        self.c_spec.setData(f, amp[:, 0])
        self.p_spec.setTitle(f"{'XYZ'[ax]} vibration amplitude spectrum (Welch, Hann)")
        nyq = fs / 2
        vmax = sc.valid_max if sc.spectral_ok else 0.0
        self.invalid_region.setRegion((vmax, nyq))
        self.invalid_lbl.setPos(vmax, float(np.log10(amp.max())))
        self.dlpf_line.setValue(sc.dlpf_bw)
        self.p_spec.setXRange(0, nyq, padding=0.01)
        fmin = float(self.session.cfg.get("fft_fmin_hz", 2))
        pk = dominant_peak(f, amp[:, 0], fmin, vmax)
        if pk.valid:
            self.peak_marker.setData([pk.freq_hz], [pk.amp])
        else:
            self.peak_marker.setData([], [])
        dev = self.session.spectrum
        dev_txt = "—"
        if dev:
            dev_txt = (f"{dev['dom_hz']:.2f} Hz, {dev['dom_amp']:.2f} m/s² "
                       f"({'valid' if dev['valid'] else 'NOT valid'})")
        host_txt = (f"{pk.freq_hz:.2f} Hz, {pk.amp:.2f} m/s² (noise floor {pk.noise_floor:.2f})"
                    if pk.valid else "no peak >= 12 dB above the band median in the validated band")
        self.spec_info.setText(
            f"fs = {fs:.2f} Hz   resolution = {fs / nseg:.2f} Hz   Nyquist = {nyq:.2f} Hz   "
            f"DLPF = {sc.dlpf_bw:.2f} Hz   validated ≤ {vmax:.2f} Hz\n"
            f"Dominant ({'XYZ'[ax]}, {fmin:.2f}–{vmax:.2f} Hz) host: {host_txt}\n"
            f"Dominant device FFT: {dev_txt}\n"
            f"Grey band: not a valid measurement (DLPF roll-off / aliasing). "
            f"Target band {TARGET_BAND_HZ[0]:.2f}–{TARGET_BAND_HZ[1]:.2f} Hz is "
            f"{'covered' if sc.target_band_covered else 'NOT measurable with this sensor configuration'}.")

    # --------------------------------------------------------- slow panels
    def _update_slow(self) -> None:
        s = self.session
        now = time.monotonic()
        if now - self._rate_t >= 1.0:
            self.line_rate = self._rate_lines / (now - self._rate_t)
            self._rate_t, self._rate_lines = now, 0
        if self.link and s.cfg is None and now - self.connected_at > 3 and now - self.last_hello_req > 2.5:
            self.last_hello_req = now
            self._send("HELLO")

        st = s.status
        if st:
            name = "XYZ"[s.cfg.primary_axis] if s.cfg else "Z"
            v = st["a" + name.lower()]
            self.v_acc["m/s²"].setText(f"{v:9.2f}")
            self.v_acc["g"].setText(f"{v / G:9.2f}")
            if s.cfg:
                tz = s.cfg.tare[s.cfg.primary_axis]
                self.v_acc["Zero offset"].setText(f"{tz:.2f} m/s²" if tz else "none (incl. gravity)")
            u, us = self.uscale, "g" if self.units_g else "m/s²"
            self.v_vib["RMS"].setText(f"{st['rms_win'] * u:.2f} {us}")
            self.v_vib["Peak-to-peak"].setText(f"{st['p2p_win'] * u:.2f} {us}")
            self.v_vib["Peak (interval)"].setText(f"{st['peak_int'] * u:.2f} {us}")
            self.v_test["Max |dz|"].setText(f"{st['test_max_dyn'] * u:.2f} {us}")
            self.v_test["RMS"].setText(f"{st['test_rms'] * u:.2f} {us}")
            self.v_test["Peak-to-peak"].setText(f"{st['test_p2p'] * u:.2f} {us}")
            self.v_test["Impacts"].setText(f"{st['impacts']}{'  (active)' if st['impact_active'] else ''}")
            flags = int(st["flags"])
            items = []
            for bit, (name, desc) in FLAGS.items():
                if flags & (1 << bit) and name not in ("SENSOR_OK", "RUNNING"):
                    items.append(f"<span style='color:#ff6b6b'>{name}</span>: {desc}"
                                 if (1 << bit) & FLAG_PROBLEMS else f"{name}: {desc}")
            state = ("running" if flags & 2 else "stopped") if flags & 1 else "SENSOR NOT OK"
            self.flags_lbl.setText(f"<b>Device:</b> {state}<br>" + "<br>".join(items))
        if s.test:
            self.v_test["Clipped samples"].setText(str(s.test["clipped"]))
        sp = s.spectrum
        if sp:
            self.v_vib["Dominant freq"].setText(
                f"{sp['dom_hz']:.2f} Hz" if sp["valid"] else "not valid")

        sc = s.sampling()
        if s.cfg:
            self.v_samp["Nominal ODR"].setText(f"{sc.odr_nominal:.2f} Hz")
            self.v_samp["Measured fs"].setText(f"{sc.fs_measured:.2f} Hz" if sc.fs_measured else "measuring…")
            self.v_samp["Deviation"].setText(f"{sc.deviation_pct:+.2f} %")
            self.v_samp["DLPF bandwidth"].setText(f"{sc.dlpf_bw:.2f} Hz")
            self.v_samp["Nyquist"].setText(f"{sc.nyquist:.2f} Hz")
            self.v_samp["Validated band"].setText(f"≤ {sc.valid_max:.2f} Hz" if sc.valid_max else "none")
            self.v_samp["Stream missing"].setText(f"{s.processor.missing} ({sc.missing_pct:.2f} %)")
            self.v_samp["Duplicates"].setText(f"{sc.dup_ratio * 100:.2f} %")
            if st:
                self.v_samp["FIFO ovf / I2C err"].setText(f"{st['fifo_ovf']} / {st['i2c_err']}")
            self.v_samp["Spectral data"].setText("VALID" if sc.spectral_ok else "NOT VALID")
            col = "#51cf66" if sc.spectral_ok else "#ff922b"
            self.samp_verdict.setText(f"<span style='color:{col}'>{sc.verdict}</span>")
            self.target_lbl.setText(
                f"<b>Target {TARGET_BAND_HZ[0]:.2f}–{TARGET_BAND_HZ[1]:.2f} Hz:</b> "
                + ("covered" if sc.target_band_covered else
                   "<span style='color:#ff922b'>not measurable</span> — the MPU6500 accelerometer bandwidth "
                   "over 400 kHz I²C is limited to the validated band above (README: Bandwidth and frequency limits)."))
            self.cal_current_lbl.setText(
                "Device calibration: offset [m/s²] " + " ".join(f"{v:+.2f}" for v in s.cfg.offset)
                + "   scale " + " ".join(f"{v:.2f}" for v in s.cfg.scale))

        age = now - s.last_status_time if s.last_status_time else None
        conn = f"connected: {self.link.name}" if self.link else "disconnected"
        txt = f"{conn}   {self.line_rate:.2f} lines/s   samples {s.raw_count}"
        if self.link and (age is None or age > 2.0):
            txt += "   <b>NO DATA</b> (check port / baud / firmware)"
        if self.link:
            txt += f"   parse errors {self.link.parser.errors}   queue drops {self.link.dropped}"
        self.status_lbl.setText(txt)
        rec = s.recorder
        self.rec_lbl.setText(f"● REC {rec.samples} samples, {len(rec.events)} events → {rec.path}"
                             if rec.active else "")
        self.dirty_lbl.setText("<span style='color:#ff922b'>unsent changes — press Apply</span>"
                               if self._settings_dirty else "")

    # ------------------------------------------------------------ settings
    def _mark_dirty(self, *_) -> None:
        if not getattr(self, "_loading", False):
            self._settings_dirty = True

    def _load_settings(self, cfg) -> None:
        self._loading = True
        try:
            for key, ckey, _, kind, *_ in DEVICE_PARAMS:
                w = self.param_w[key]
                v = cfg.get(ckey)
                if v is None:
                    continue
                if kind == "combo":
                    w.setCurrentText(str(v))
                elif kind == "float":
                    w.setValue(float(v))
                elif kind == "int":
                    w.setValue(int(v))
                else:
                    w.setChecked(bool(int(v)))
        finally:
            self._loading = False
        self._settings_dirty = False

    def _widget_value(self, key: str, kind: str) -> str:
        w = self.param_w[key]
        if kind == "combo":
            return w.currentText()
        if kind == "float":
            return f"{w.value():g}"
        if kind == "int":
            return str(w.value())
        return "1" if w.isChecked() else "0"

    def _apply_settings(self) -> None:
        cfg = self.session.cfg
        if cfg is None:
            self.session._log("no device configuration yet - connect first")
            return
        cmds = []
        for key, ckey, _, kind, *_ in DEVICE_PARAMS:
            new = self._widget_value(key, kind)
            old = cfg.get(ckey)
            same = (abs(float(new) - float(old)) < 1e-6) if kind in ("float", "int", "check") else str(old) == new
            if not same:
                cmds.append((key, new))
        if not cmds:
            self.session._log("settings unchanged")
            return
        # Keep hysteresis < threshold during the update: raise threshold first,
        # lower it last.
        thr_new = self.param_w["THRESHOLD"].value()
        order = {"THRESHOLD": 0 if thr_new > float(cfg.get("threshold")) else 2, "HYSTERESIS": 1}
        cmds.sort(key=lambda kv: order.get(kv[0], 1))
        for key, val in cmds:
            self._send(f"SET {key} {val}")
        self._settings_dirty = False

    # ----------------------------------------------------------- recording
    def _toggle_record(self, on: bool) -> None:
        if on:
            if self.session.cfg is None:
                self.session._log("cannot record: no device configuration yet")
                self.rec_btn.setChecked(False)
                return
            if self.reset_on_rec.isChecked():
                self._send("RESET")
            self.session.start_recording(self.label_edit.text())
            self.rec_btn.setText("■ Stop recording")
        else:
            path = self.session.stop_recording()
            self.rec_btn.setText("● Record")
            if path:
                self.statusBar().showMessage(f"Saved {path}", 10000)

    def _export_snapshot(self) -> None:
        if self.session.buffer.n == 0:
            self.session._log("nothing to export yet")
            return
        path = self.session.export_snapshot(self.label_edit.text())
        self.statusBar().showMessage(f"Exported {path}", 10000)

    # -------------------------------------------------------------- events
    def _add_event_row(self, rec: EventRecord) -> None:
        r, ev, fs = rec.result, rec.ev, self.session.fs
        row = self.evt_table.rowCount()
        self.evt_table.insertRow(row)
        vals = [str(ev["id"]), f"{ev['start_idx'] / fs:.2f}", fmt(ev["peak_dyn"]), fmt(ev["peak_dyn"] / G),
                fmt(ev["duration_ms"]), fmt(r.compression_peak), fmt(r.rebound_peak),
                fmt(r.rebound_ratio), fmt(r.damped_freq_hz), fmt(r.damping_ratio),
                fmt(r.settling_ms), fmt(r.fit_r2), "; ".join(r.notes)]
        for c, v in enumerate(vals):
            item = QtWidgets.QTableWidgetItem(v)
            item.setData(Qt.ItemDataRole.UserRole, id(rec))
            self.evt_table.setItem(row, c, item)
        while self.evt_table.rowCount() > self.session.s.max_events:
            self.evt_table.removeRow(0)
        if self.follow_chk.isChecked():
            self.selected_event = rec
            self._plot_event(rec)
            self.evt_table.scrollToBottom()

    def _event_selected(self) -> None:
        items = self.evt_table.selectedItems()
        if not items:
            return
        key = items[0].data(Qt.ItemDataRole.UserRole)
        for rec in self.session.events:
            if id(rec) == key:
                self.follow_chk.setChecked(False)
                self.selected_event = rec
                self._plot_event(rec)
                return

    def _plot_event(self, rec: EventRecord) -> None:
        r = rec.result
        if r is None:
            return
        u = self.uscale
        us = "g" if self.units_g else "m/s²"
        self.p_evt.setLabel("left", us)
        self.c_evt.setData(r.t, r.x * u)
        if r.x_fit is not r.x and len(r.x_fit) == len(r.t):
            self.c_evt_f.setData(r.t, r.x_fit * u)
        else:
            self.c_evt_f.setData([], [])
        self.s_lobes.setData(r.lobe_t, r.lobe_a * u)
        if math.isfinite(r.decay_rate_per_s) and len(r.lobe_t):
            te = np.linspace(r.lobe_t[0], r.t[-1], 200)
            A0 = abs(r.lobe_a[0]) * np.exp(r.decay_rate_per_s * r.lobe_t[0])
            env = A0 * np.exp(-r.decay_rate_per_s * te) * u
            self.c_env_p.setData(te, env)
            self.c_env_n.setData(te, -env)
        else:
            self.c_env_p.setData([], [])
            self.c_env_n.setData([], [])
        ev = rec.ev
        self.evt_info.setText(
            f"Event #{ev['id']}: peak |dz| {ev['peak_dyn'] * u:.2f} {us}  duration {ev['duration_ms']:.2f} ms"
            f"{'  (TRUNCATED)' if ev['truncated'] else ''}\n"
            f"Compression {fmt(r.compression_peak * u)}  rebound {fmt(r.rebound_peak * u)} {us}  "
            f"rebound/compression {fmt(r.rebound_ratio)}  Δt {fmt(r.comp_to_rebound_ms)} ms\n"
            f"Ring-down: {r.lobes} half-cycles  fd {fmt(r.damped_freq_hz)} Hz"
            f"{'' if r.freq_valid or not math.isfinite(r.damped_freq_hz) else ' (NOT validated)'}  "
            f"σ {fmt(r.decay_rate_per_s)} 1/s  ζ {fmt(r.damping_ratio)}  "
            f"log-dec {fmt(r.log_decrement)}  R² {fmt(r.fit_r2)}  settling {fmt(r.settling_ms)} ms\n"
            f"Method: {r.method}.  {'; '.join(r.notes)}")

    # --------------------------------------------------------- calibration
    def _cal_capture(self, pos: Optional[str]) -> None:
        if not self.link:
            self.session._log("connect first")
            return
        # A new CAL restarts the device's capture, which would silently drop the
        # pending result, so only one capture at a time (10 s timeout as fallback).
        if self.cal_pending and time.monotonic() - self.cal_started < 10.0:
            self.session._log(f"calibration capture {self.cal_pending} still in progress - wait for it")
            return
        self.cal_pending = pos or "STATIONARY"
        self.cal_started = time.monotonic()
        if isinstance(self.link, SimulatedLink) and pos:
            self._send(f"SIM_ORIENT {pos}")
        self._send("CAL 3000")
        if pos:
            self.cal_pos_lbl[pos].setText("capturing… keep still")
        else:
            self.cal_station_lbl.setText("capturing… keep still")

    def _on_cal_result(self, msg: dict) -> None:
        pos, self.cal_pending = self.cal_pending, None
        cfg = self.session.cfg
        rep = calibration.stationary_report(msg, cfg.offset if cfg else np.zeros(3),
                                            cfg.scale if cfg else np.ones(3))
        if pos in (None, "STATIONARY"):
            col = "#51cf66" if rep.ok else "#ff922b"
            self.cal_station_lbl.setText(f"<span style='color:{col}'>{rep.text}</span>")
            return
        cap = calibration.CalCapture(
            np.array([msg["mean_x"], msg["mean_y"], msg["mean_z"]], float),
            np.array([msg["std_x"], msg["std_y"], msg["std_z"]], float), int(msg["samples"]))
        self.cal_caps[pos] = cap
        warn = calibration.check_capture(pos, cap)
        self.cal_pos_lbl[pos].setText(
            "mean " + " ".join(f"{v:+8.2f}" for v in cap.mean) + "  std " + " ".join(f"{v:.2f}" for v in cap.std)
            + ("   ⚠ " + "; ".join(warn) if warn else "   ✓"))

    def _cal_compute(self) -> None:
        sol = calibration.solve(self.cal_caps)
        if sol is None:
            missing = [p for p in calibration.POSITIONS if p not in self.cal_caps]
            self.cal_result_lbl.setText("Missing captures: " + ", ".join(missing))
            return
        self.cal_solution = sol
        res = "  ".join(f"{p}:{v:+.2f}" for p, v in sol.residual_g.items())
        self.cal_result_lbl.setText(
            "offset [m/s²] " + " ".join(f"{v:+.2f}" for v in sol.offset)
            + "\nscale        " + " ".join(f"{v:.2f}" for v in sol.scale)
            + f"\n|g| residual after calibration [m/s²]: {res}"
            + ("\n⚠ " + "\n⚠ ".join(sol.warnings) if sol.warnings else "\n✓ captures consistent"))

    def _cal_apply(self) -> None:
        if not self.cal_solution:
            self.cal_result_lbl.setText("Compute first.")
            return
        for c in self.cal_solution.commands():
            self._send(c)
        if isinstance(self.link, SimulatedLink):
            self._send("SIM_BUMPS 1")
        self.session._log("calibration sent - press 'Save to flash' to keep it after power-off")

    def _clear_tare(self) -> None:
        ax = "XYZ"[self.session.cfg.primary_axis] if self.session.cfg else "Z"
        self._send(f"SET TARE_{ax} 0")
        self._send("SAVE")

    def _cal_clear(self) -> None:
        for a in "XYZ":
            self._send(f"SET OFS_{a} 0")
            self._send(f"SET SCL_{a} 1")

    def _cal_reset(self) -> None:
        self.cal_caps.clear()
        self.cal_solution = None
        for lbl in self.cal_pos_lbl.values():
            lbl.setText("not captured")
        self.cal_result_lbl.setText("")

    # ------------------------------------------------------------- closing
    def closeEvent(self, ev) -> None:  # noqa: N802 (Qt API)
        if self.session.recorder.active:
            self.session.stop_recording()
        if self.link:
            self.link.stop()
        super().closeEvent(ev)


def resource_path(*parts: str) -> str:
    """Path to a bundled file, both from source and inside the PyInstaller build."""
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(base, *parts)


def user_data_dir() -> str:
    """~/Documents/ShockMonitor (installed app: the program folder is not writable)."""
    return os.path.join(os.path.expanduser("~"), "Documents", "ShockMonitor")


def default_record_dir() -> str:
    if getattr(sys, "frozen", False):          # packaged .exe
        return os.path.join(user_data_dir(), "recordings")
    return os.path.join(os.getcwd(), "recordings")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Shock absorber monitoring dashboard")
    ap.add_argument("--port", help="serial port (e.g. /dev/cu.usbserial-0001, COM5)")
    ap.add_argument("--baud", type=int, default=921600, help="must match SERIAL_BAUD in config.h")
    ap.add_argument("--simulate", action="store_true", help="use the built-in device simulator")
    ap.add_argument("--record-dir", default=default_record_dir())
    ap.add_argument("--buffer-s", type=float, default=60.0, help="seconds of history kept in memory")
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    win = MainWindow(args)
    win.show()
    run = getattr(app, "exec", None) or app.exec_
    return run()
