"""Entry point for the setup wizard, the installed uninstaller and app updates."""

import argparse
import sys
from pathlib import Path


def main() -> int:
    if "--smoke-test" in sys.argv:
        # Lets CI prove that the frozen build bundled tkinter and CA certificates.
        import tkinter  # noqa: F401

        from gps_desktop_setup import core

        core._ssl_context()
        import certifi  # noqa: F401

        return 0 if core.ICON_PATH.is_file() else 1

    if sys.platform == "win32":
        # Keeps text sharp on high-DPI displays rather than letting Windows blur it.
        import ctypes

        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass

    from gps_desktop_setup import wizard

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--uninstall", action="store_true")
    parser.add_argument("--update", action="store_true")
    parser.add_argument("--target", type=Path)
    parser.add_argument("--wait-pid", type=int)
    args, _ = parser.parse_known_args()

    if args.uninstall:
        return wizard.run_uninstall(Path(sys.executable).resolve().parent)
    if args.update and args.target:
        return wizard.run_update(args.target, args.wait_pid)
    return wizard.run_wizard()


if __name__ == "__main__":
    sys.exit(main())
