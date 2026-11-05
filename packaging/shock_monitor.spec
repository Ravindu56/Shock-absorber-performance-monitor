# PyInstaller spec for the Shock Absorber Monitor dashboard.
#
# Build (from the project root, on the target OS - PyInstaller cannot cross-compile):
#     pyinstaller --noconfirm --clean packaging/shock_monitor.spec
# Output: dist/ShockMonitor/ShockMonitor.exe (Windows) plus its support files.
# packaging/build_windows.bat does this and then builds the installer.

import os
import sys

from PyInstaller.utils.hooks import collect_submodules

ROOT = os.path.abspath(os.path.join(SPECPATH, ".."))
ICON = os.path.join(ROOT, "packaging", "icon.ico")

a = Analysis(
    [os.path.join(ROOT, "run_dashboard.py")],
    pathex=[ROOT],
    datas=[(os.path.join(ROOT, "packaging", "icon.png"), "packaging")],
    # pyqtgraph loads parts of itself dynamically (UI templates, exporters).
    hiddenimports=["serial.tools.list_ports"] + collect_submodules(
        "pyqtgraph", filter=lambda n: not any(x in n for x in (".examples", ".opengl", ".jupyter"))),
    # Only PySide6 is bundled; other Qt bindings and heavy unused packages are excluded.
    excludes=[
        "PyQt5", "PyQt6", "PySide2", "tkinter", "matplotlib", "scipy", "pandas",
        "IPython", "jupyter", "notebook", "pytest", "PIL",
        "dashboard.tests",
    ],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="ShockMonitor",
    icon=ICON,
    console=False,          # GUI app: no console window (errors go to Documents/ShockMonitor/crash.log)
    upx=False,
)

coll = COLLECT(exe, a.binaries, a.datas, name="ShockMonitor", upx=False)

if sys.platform == "darwin":  # lets the same spec be smoke-tested on macOS
    app = BUNDLE(coll, name="ShockMonitor.app", icon=None, bundle_identifier="local.shockmonitor")
