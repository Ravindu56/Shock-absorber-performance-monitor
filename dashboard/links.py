"""Device connections: a real serial port or the built-in simulator.

Both run a background thread that parses lines into ``protocol.Message``
objects and puts them on ``rx`` (a queue drained by the GUI timer), so a
slow GUI frame never stalls serial reception.
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from typing import List, Optional, Tuple

import serial
import serial.tools.list_ports

from .protocol import Message, Parser
from .simulator import DeviceSimulator

RX_QUEUE_MAX = 50000  # messages; ~25 s of data if the GUI stops draining


# USB-to-UART bridges used on ESP32 dev boards: (VID, PID) -> name
ESP32_USB_IDS = {
    (0x10C4, 0xEA60): "CP210x",            # Silicon Labs CP2102/CP2104 (DOIT DevKit V1)
    (0x1A86, 0x7523): "CH340",
    (0x1A86, 0x55D4): "CH9102",
    (0x1A86, 0x55D3): "CH343",
    (0x0403, 0x6001): "FTDI FT232",
    (0x0403, 0x6015): "FTDI FT231X",
    (0x303A, 0x1001): "ESP32 USB-JTAG",
}


def list_ports() -> List[str]:
    return [dev for dev, _ in describe_ports()]


def describe_ports() -> List[Tuple[str, str]]:
    """(device, label) for every usable serial port, likely ESP32 boards first."""
    esp, other = [], []
    seen = set()
    for p in sorted(serial.tools.list_ports.comports(), key=_port_preference):
        # Hide Bluetooth/debug pseudo-ports that macOS exposes.
        if any(s in p.device for s in ("Bluetooth", "debug-console", "wlan-debug")):
            continue
        chip = ESP32_USB_IDS.get((p.vid, p.pid))
        if chip:
            # macOS can expose one adapter twice (cu.usbserial-* and cu.SLAB_USBtoUART):
            # keep one device per physical adapter.
            key = (p.vid, p.pid, p.serial_number, p.location)
            if key in seen:
                continue
            seen.add(key)
            esp.append((p.device, f"{p.device}  —  ESP32 ({chip})"))
        else:
            other.append((p.device, f"{p.device}  —  {p.description or 'serial port'}"))
    return esp + other


def find_esp32_ports() -> List[str]:
    """Ports whose USB IDs match an ESP32 dev-board USB-UART bridge."""
    return [dev for dev, label in describe_ports() if "ESP32 (" in label]


def _port_preference(p) -> Tuple[int, str]:
    # Prefer the standard driver names (cu.usbserial*, cu.wchusbserial*, COMx).
    return (0 if ("usbserial" in p.device or p.device.upper().startswith("COM")) else 1, p.device)


class Link:
    name = "link"

    def __init__(self) -> None:
        self.rx: "queue.Queue[Message]" = queue.Queue(maxsize=RX_QUEUE_MAX)
        self.parser = Parser()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.lines = 0
        self.bytes = 0
        self.dropped = 0          # messages lost because the queue was full
        self.last_rx = 0.0        # time.monotonic() of the last received line

    def _put(self, msg: Message) -> None:
        try:
            self.rx.put_nowait(msg)
        except queue.Full:
            self.dropped += 1

    def _handle_line(self, line: str) -> None:
        self.lines += 1
        self.bytes += len(line) + 1
        self.last_rx = time.monotonic()
        msg = self.parser.parse(line)
        if msg is not None:
            self._put(msg)

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._thread = None

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def send(self, cmd: str) -> None:
        raise NotImplementedError

    def _run(self) -> None:
        raise NotImplementedError


class SerialLink(Link):
    def __init__(self, port: str, baud: int = 921600) -> None:
        super().__init__()
        self.port, self.baud = port, baud
        self.name = port
        self._ser: Optional[serial.Serial] = None
        self._wlock = threading.Lock()

    def start(self) -> None:
        # Raises serial.SerialException to the caller if the port can't be opened.
        # exclusive: refuse to share the port (e.g. with an open serial monitor),
        # which would split the data between two readers. Windows ports are always exclusive.
        kw = {} if sys.platform.startswith("win") else {"exclusive": True}
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.5, **kw)
        super().start()

    def stop(self) -> None:
        super().stop()
        if self._ser:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
        self._ser = None

    def send(self, cmd: str) -> None:
        if not self._ser:
            return
        try:
            with self._wlock:
                self._ser.write((cmd.strip() + "\n").encode("ascii"))
        except (serial.SerialException, OSError) as exc:
            self._put(Message("LINK", f"write failed: {exc}"))

    def _run(self) -> None:
        buf = bytearray()
        ser = self._ser
        synced = False   # drop the partial line that was in flight when the port opened
        errors = 0
        while not self._stop.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
                errors = 0
            except (serial.SerialException, OSError) as exc:
                # Opening the port resets the ESP32 and some drivers report a spurious
                # error meanwhile; only a persistent error means the cable was pulled.
                errors += 1
                if errors >= 10:
                    self._put(Message("LINK", f"serial error: {exc} - disconnected"))
                    return
                time.sleep(0.1)
                continue
            if not chunk:
                continue
            buf.extend(chunk)
            if not synced:
                nl = buf.find(b"\n")
                if nl < 0:
                    continue
                del buf[:nl + 1]
                synced = True
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw = bytes(buf[:nl])
                del buf[:nl + 1]
                # Bootloader messages at 115200 baud arrive as garbage: decode leniently.
                self._handle_line(raw.decode("ascii", errors="replace").rstrip("\r"))
            if len(buf) > 8192:  # no newline in 8 kB: wrong baud rate or noise
                buf.clear()
                self._put(Message("LINK", "discarded 8 kB without line ending - check baud rate"))


class SimulatedLink(Link):
    name = "simulator"

    def __init__(self, seed: Optional[int] = None) -> None:
        super().__init__()
        self.sim = DeviceSimulator(seed=seed)
        self._cmds: "queue.Queue[str]" = queue.Queue()

    def send(self, cmd: str) -> None:
        self._cmds.put(cmd)

    def _run(self) -> None:
        for line in ["# simulator booting", "# no saved configuration - using defaults"]:
            self._handle_line(line)
        self.sim.hello()
        period = 0.02
        t_last = time.monotonic()
        while not self._stop.is_set():
            while not self._cmds.empty():
                self.sim.command(self._cmds.get_nowait())
            now = time.monotonic()
            dt, t_last = now - t_last, now
            for line in self.sim.step(min(dt, 0.2)):
                self._handle_line(line)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
