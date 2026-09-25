"""Entry point: python main.py [--simulated] [--theme dark|light]."""
from __future__ import annotations

import argparse
import sys

from PyQt6.QtWidgets import QApplication

from ui.main_window import MainWindow


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Jib crane data acquisition GUI")
    ap.add_argument("--simulated", action="store_true", help="preselect the simulated device")
    ap.add_argument("--theme", choices=("dark", "light"), default="dark")
    args = ap.parse_args(argv)

    app = QApplication(sys.argv[:1])
    app.setApplicationName("Jib Crane DAQ")
    win = MainWindow(theme=args.theme)
    if args.simulated:
        win.conn.select_simulated()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
