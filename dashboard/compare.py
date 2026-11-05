"""Compare recorded test sessions (performance comparison between prototypes).

    python -m dashboard.compare recordings/*            # table on the console
    python -m dashboard.compare recordings/* -o comparison.csv

Reads summary.csv from each recording folder and lines up the key metrics:
event peak, rebound ratio, damped frequency, damping ratio and settling time
(mean +- std over the events of a test), plus whole-test vibration statistics
and the sampling verification, so tests taken under different sensor settings
are not compared blindly.
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from typing import Dict, List

from .recorder import read_summary

COLUMNS = [
    ("label", "label", ""),
    ("started", "started", ""),
    ("profile", "cfg_profile", ""),
    ("range_g", "cfg_range_g", ""),
    ("thr_ms2", "cfg_threshold", ""),
    ("fs_Hz", "fs_measured_hz", ".1f"),
    ("spectral_ok", "spectral_ok", ""),
    ("missing_%", "missing_pct", ".2f"),
    ("events", "events", ""),
    ("peak_ms2", "peak_dyn_ms2", "pm"),
    ("compr_ms2", "compression_peak_ms2", "pm"),
    ("reb_ratio", "rebound_ratio", "pm"),
    ("fd_Hz", "damped_freq_hz", "pm"),
    ("zeta", "damping_ratio", "pm"),
    ("settle_ms", "settling_ms", "pm"),
    ("test_rms", "test_rms", ".3f"),
    ("test_max", "test_max_dyn", ".2f"),
    ("test_p2p", "test_p2p", ".2f"),
    ("clipped", "test_clipped", ""),
]


def _f(v: str, spec: str) -> str:
    if v in (None, ""):
        return "—"
    try:
        return format(float(v), spec) if spec else v
    except ValueError:
        return v


def row_for(summary: Dict[str, str]) -> Dict[str, str]:
    out = {}
    for name, key, spec in COLUMNS:
        if spec == "pm":
            m, s = summary.get(f"{key}_mean", ""), summary.get(f"{key}_std", "")
            out[name] = "—" if m == "" else f"{float(m):.3g} ± {float(s or 0):.2g}"
        else:
            out[name] = _f(summary.get(key, ""), spec)
    return out


def find_summaries(paths: List[str]) -> List[str]:
    found = []
    for p in paths:
        for q in glob.glob(p) or [p]:
            f = q if q.endswith("summary.csv") else os.path.join(q, "summary.csv")
            if os.path.isfile(f):
                found.append(f)
    return sorted(set(found))


def main(argv: List[str] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="recording folders (or their summary.csv files)")
    ap.add_argument("-o", "--output", help="write the comparison table to this CSV file")
    args = ap.parse_args(argv)

    files = find_summaries(args.paths)
    if not files:
        print("no summary.csv found (stop a recording to create one)", file=sys.stderr)
        return 1
    rows = []
    for f in files:
        r = row_for(read_summary(f))
        r["folder"] = os.path.basename(os.path.dirname(f))
        rows.append(r)

    names = [c[0] for c in COLUMNS]
    widths = {n: max(len(n), *(len(str(r[n])) for r in rows)) for n in names}
    print("  ".join(n.ljust(widths[n]) for n in names))
    for r in rows:
        print("  ".join(str(r[n]).ljust(widths[n]) for n in names))

    if len({r["profile"] for r in rows}) > 1:
        print("\nNOTE: tests use different sensor profiles - frequency and peak values are not "
              "directly comparable (different bandwidth).")
    if any(r["spectral_ok"] not in ("1", "—") for r in rows):
        print("NOTE: some tests had unverified sampling (spectral_ok = 0) - treat their frequencies with care.")

    if args.output:
        with open(args.output, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["folder"] + names)
            w.writeheader()
            w.writerows(rows)
        print(f"\nwritten {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
