"""Checks that the dashboard parser and the firmware source agree on the protocol."""

import os
import re
import unittest

from dashboard.protocol import DEFAULT_HEADERS

SRC = os.path.join(os.path.dirname(__file__), "..", "..", "src", "serial_protocol.cpp")


def emit_calls(body: str):
    """Yield the concatenated format string of each emitf(...) call."""
    for m in re.finditer(r"emitf\((.*?)\);", body, re.S):
        lits = re.findall(r'"((?:[^"\\]|\\.)*)"', m.group(1))
        # the format string is the first run of adjacent literals
        fmt, rest = "", m.group(1)
        for lit in lits:
            fmt += lit
            idx = rest.index('"' + lit + '"') + len(lit) + 2
            rest = rest[idx:]
            if not re.match(r"\s*\"", rest):
                break
        yield fmt


def function_body(src: str, name: str) -> str:
    start = src.index(name + "(")
    brace = src.index("{", start)
    depth, i = 0, brace
    while True:
        depth += {"{": 1, "}": -1}.get(src[i], 0)
        if depth == 0:
            return src[brace:i]
        i += 1


@unittest.skipUnless(os.path.isfile(SRC), "firmware source not found")
class FirmwareSyncTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(SRC, encoding="utf-8") as f:
            cls.src = f.read()

    def test_header_lists_match(self):
        fw = {}
        for fmt in emit_calls(function_body(self.src, "void sendHeaders")):
            parts = fmt.split(",")
            fw[parts[1]] = parts[2:]
        for kind, names in DEFAULT_HEADERS.items():
            self.assertEqual(fw.get(kind), names, f"H,{kind} differs from firmware")

    def test_field_counts_match_headers(self):
        for kind, func in [("S", "void sendStatus"), ("TS", "void sendTestStats"), ("E", "void sendEvent"),
                           ("F", "void sendSpectrum"), ("CAL", "void sendCalibration")]:
            fmt = next(emit_calls(function_body(self.src, func)))
            self.assertTrue(fmt.startswith(kind + ","), fmt)
            n = len(re.findall(r"%[-+ #0-9.]*[a-zA-Z]", fmt))
            self.assertEqual(n, len(DEFAULT_HEADERS[kind]), f"{kind}: {n} fields vs header")


if __name__ == "__main__":
    unittest.main()
