from __future__ import annotations

import sys

from PyQt6.QtWidgets import QApplication

from ui.main_window import MainWindow


def main(argv=None) -> int:
    app = QApplication(sys.argv[:1])
    app.setApplicationName("Grua Pluma Cascade Control")
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
