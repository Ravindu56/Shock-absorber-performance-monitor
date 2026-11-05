"""Launcher for the dashboard: IDE "Run" button and the packaged Windows .exe.

Equivalent to `python3 -m dashboard` (arguments are passed through, e.g.
`python3 run_dashboard.py --simulate`). The modules inside dashboard/ use
package-relative imports and cannot be run directly as scripts.
"""

import datetime
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FROZEN = getattr(sys, "frozen", False)
if FROZEN:
    os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")  # the binding bundled in the .exe


def _log_crash(text: str) -> str:
    """Write the error to Documents/ShockMonitor/crash.log (a windowed .exe has no console)."""
    folder = os.path.join(os.path.expanduser("~"), "Documents", "ShockMonitor")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, "crash.log")
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"\n=== {datetime.datetime.now():%Y-%m-%d %H:%M:%S} ===\n{text}")
    return path


def _show_error(text: str) -> None:
    path = _log_crash(text)
    try:
        from pyqtgraph.Qt import QtWidgets
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        QtWidgets.QMessageBox.critical(None, "Shock Absorber Monitor - error",
                                       f"{text.strip().splitlines()[-1]}\n\nDetails saved to:\n{path}")
        del app
    except Exception:  # Qt itself failed to load: the log file is all we can do
        pass


def _excepthook(exc_type, exc, tb) -> None:
    # Errors inside Qt callbacks: log them and keep the app running.
    text = "".join(traceback.format_exception(exc_type, exc, tb))
    _log_crash(text)
    sys.__stderr__ and sys.__stderr__.write(text)


if __name__ == "__main__":
    if FROZEN:
        sys.excepthook = _excepthook
    try:
        from dashboard.app import main
        sys.exit(main())
    except SystemExit:
        raise
    except Exception:
        _show_error(traceback.format_exc())
        sys.exit(1)
