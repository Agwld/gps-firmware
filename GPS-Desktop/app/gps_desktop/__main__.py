"""Starts the app: splash screen, update check, then the main window."""

import argparse
import sys
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication

from gps_desktop import splash, theme
from gps_desktop.ui.main_window import MainWindow


def _screenshots(app: QApplication, window: MainWindow, folder: Path, seconds: float) -> None:
    """Save a picture of every screen once some data has arrived, then quit."""

    def capture() -> None:
        folder.mkdir(parents=True, exist_ok=True)
        for index, screen in enumerate(window.screens):
            window.show_screen(index)
            app.processEvents()
            window.grab().save(str(folder / f"{index + 1}-{screen.nav_label.lower()}.png"))
        app.quit()

    QTimer.singleShot(int(seconds * 1000), capture)


def main() -> int:
    parser = argparse.ArgumentParser(prog="GPSDesktop")
    parser.add_argument("--simulate", action="store_true", help="start connected to the simulated GPS node")
    parser.add_argument("--screenshots", type=Path, metavar="FOLDER",
                        help="save a picture of every screen to this folder, then quit (for checking a build)")
    parser.add_argument("--after", type=float, default=6.0, metavar="SECONDS",
                        help="how long to collect data before --screenshots (default 6)")
    parser.add_argument("--smoke-test", action="store_true", help=argparse.SUPPRESS)
    args, _ = parser.parse_known_args()

    app = QApplication(sys.argv)
    app.setApplicationName("GPS Desktop")
    icon = QIcon(str(Path(__file__).parent / "assets" / "icon.png"))
    app.setWindowIcon(icon)
    theme.apply(app)

    window = MainWindow()

    # Lets CI prove that the frozen build can start Qt, read the CAN database, decode simulated
    # frames, load a CAN backend and check for updates.
    if args.smoke_test:
        if icon.isNull():
            return 1
        import can.interfaces.virtual  # noqa: F401
        import pyqtgraph  # noqa: F401

        from gps_desktop import update_ui  # noqa: F401
        from gps_desktop.gps.simulator import Simulator
        from gps_desktop_setup import core

        core._ssl_context()
        import certifi  # noqa: F401

        simulator = Simulator(window.ctx.db, seed=1)
        frames = [frame for tick in range(200) for frame in simulator.step(tick, tick * 0.01)]
        if any(window.ctx.db.decode(frame) is None for frame in frames):
            return 1
        window.show()
        QTimer.singleShot(0, app.quit)
        return app.exec()

    if args.simulate or args.screenshots:
        window._connect(window.ctx.connect_simulator)
    if args.screenshots:
        window.show()
        _screenshots(app, window, args.screenshots, args.after)
        return app.exec()

    startup = splash.Splash(icon, window)
    startup.show()

    # Source runs have no installer to update them, or the installer package on their path.
    controller = None
    if getattr(sys, "frozen", False):
        from gps_desktop import update_ui

        controller = update_ui.install(window)

    if controller is None:
        startup.ready()
    else:
        # The splash stays up for the startup check. Any update prompt waits for the window.
        controller.check_finished.connect(startup.ready)
        controller.check(defer=startup.after_reveal)

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
