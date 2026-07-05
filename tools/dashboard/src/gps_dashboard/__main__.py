"""Entry point: `gps-dashboard` (or `python -m gps_dashboard`).

The --screenshot/--run-seconds/--simulate flags exist purely to let this
be verified headlessly/automatically (start in simulate mode, let real
data flow, grab a PNG, exit) without a human needing to click through
the connection panel by hand every time.
"""

from __future__ import annotations

import argparse
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from gps_dashboard.can_sources.simulate import SimulateCanSource
from gps_dashboard.decoder import Decoder
from gps_dashboard.ui.main_window import MainWindow


def main() -> int:
    parser = argparse.ArgumentParser(prog="gps-dashboard")
    parser.add_argument(
        "--simulate", action="store_true", help="start immediately in simulate mode"
    )
    parser.add_argument(
        "--sim-lap-period",
        type=float,
        default=20.0,
        help="simulate mode: seconds per lap (shorter = laps complete sooner)",
    )
    parser.add_argument(
        "--run-seconds",
        type=float,
        default=None,
        help="exit automatically after this many seconds (for scripted verification)",
    )
    parser.add_argument(
        "--screenshot",
        metavar="PATH",
        default=None,
        help="save a PNG of the window before exiting (implies --run-seconds if not given)",
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    decoder = Decoder()
    window = MainWindow(decoder)
    window.show()

    if args.simulate:
        window._start_ingest(  # noqa: SLF001
            SimulateCanSource(decoder, lap_period_s=args.sim_lap_period), None
        )

    run_seconds = args.run_seconds
    if args.screenshot and run_seconds is None:
        run_seconds = 2.0

    if args.screenshot:
        def take_screenshot() -> None:
            window.grab().save(args.screenshot)

        QTimer.singleShot(max(0, int((run_seconds or 0) * 1000) - 200), take_screenshot)

    if run_seconds is not None:
        QTimer.singleShot(int(run_seconds * 1000), app.quit)

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
