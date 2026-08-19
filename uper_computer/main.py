from __future__ import annotations

import sys

from PyQt6.QtWidgets import QApplication

from app_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Motion Console")

    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
