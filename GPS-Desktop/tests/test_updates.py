"""Update tests against a local HTTP server standing in for GitHub. No windows are shown."""

import functools
import hashlib
import http.server
import io
import json
import os
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "installer"), str(ROOT / "app")]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from gps_desktop import updater  # noqa: E402
from gps_desktop_setup import core, system  # noqa: E402


def tar_bytes(files: dict) -> bytes:
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as tar:
        for name, content in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(content)
            info.mode = 0o755
            tar.addfile(info, io.BytesIO(content))
    return buffer.getvalue()


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


class ServerTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.site = self.root / "site"
        self.site.mkdir()
        handler = functools.partial(Quiet, directory=str(self.site))
        server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.base = f"http://127.0.0.1:{server.server_address[1]}"
        self.url = f"{self.base}/manifest.json"

    def entry(self, name: str, content: bytes) -> dict:
        (self.site / name).write_bytes(content)
        return {"name": name, "sha256": hashlib.sha256(content).hexdigest(), "size": len(content)}

    def publish(self, version, setup=("Setup.exe", b"setup"), app=None):
        manifest = {
            "version": version,
            "base_url": self.base,
            "assets": {"test-x64": self.entry("app.tar.gz", app or tar_bytes({"GPSDesktop.exe": b"new"}))},
        }
        if setup:
            manifest["setups"] = {"test-x64": self.entry(*setup)}
        (self.site / "manifest.json").write_text(json.dumps(manifest))


class VersionTests(unittest.TestCase):
    def test_comparison(self):
        for candidate, current, expected in [
            ("0.2.0", "0.1.0", True),
            ("0.10.0", "0.9.0", True),  # Numeric, not alphabetical.
            ("1.0.0", "0.99.99", True),
            ("0.1.0", "0.1.0", False),
            ("0.1.0", "0.2.0", False),
            ("0.2.0", "0.0.0-dev", False),
            ("garbage", "0.1.0", False),
        ]:
            self.assertEqual(updater.is_newer(candidate, current), expected, (candidate, current))


class FindUpdateTests(ServerTest):
    def test_newer_release_is_reported(self):
        self.publish("0.2.0")
        update = updater.find_update("0.1.0", self.url, "test-x64")
        self.assertEqual(update.version, "0.2.0")
        self.assertTrue(update.setup.url.endswith("/Setup.exe"))

    def test_same_version_is_up_to_date(self):
        self.publish("0.1.0")
        self.assertIsNone(updater.find_update("0.1.0", self.url, "test-x64"))

    def test_old_manifest_without_setups_when_up_to_date(self):
        self.publish("0.1.0", setup=None)
        self.assertIsNone(updater.find_update("0.1.0", self.url, "test-x64"))

    def test_newer_manifest_without_setups_is_an_error(self):
        self.publish("0.2.0", setup=None)
        with self.assertRaises(core.InstallError):
            updater.find_update("0.1.0", self.url, "test-x64")

    def test_offline_is_an_error(self):
        with self.assertRaises(core.InstallError):
            updater.find_update("0.1.0", "http://127.0.0.1:9/manifest.json", "test-x64")


class DownloadSetupTests(ServerTest):
    def download(self, setup):
        self.publish("0.2.0", setup=setup)
        update = updater.find_update("0.1.0", self.url, "test-x64")
        with mock.patch.object(tempfile, "tempdir", str(self.root)):
            program = updater.download_setup(update.setup)
        return program

    def test_windows_setup_is_run_directly(self):
        program = self.download(("GPSDesktopSetup-windows-x64.exe", b"exe"))
        self.assertEqual(program.read_bytes(), b"exe")

    def test_linux_setup_is_unpacked(self):
        tarball = tar_bytes({"GPSDesktopSetup": b"binary"})
        program = self.download(("GPSDesktopSetup-linux-x64.tar.gz", tarball))
        self.assertEqual(program.name, "GPSDesktopSetup")
        self.assertEqual(program.read_bytes(), b"binary")

    def test_bad_checksum_leaves_nothing_behind(self):
        self.publish("0.2.0")
        update = updater.find_update("0.1.0", self.url, "test-x64")
        bad = core.Release("0.2.0", update.setup.url, "0" * 64, update.setup.size)
        with mock.patch.object(tempfile, "tempdir", str(self.root)):
            with self.assertRaisesRegex(core.InstallError, "checksum"):
                updater.download_setup(bad)
        self.assertEqual(list(self.root.glob("gps-desktop-update-*")), [])

    def test_old_downloads_are_removed(self):
        with mock.patch.object(tempfile, "tempdir", str(self.root)):
            old = self.root / "gps-desktop-update-abc"
            old.mkdir()
            (old / "setup.exe").write_bytes(b"x")
            updater.remove_old_downloads()
        self.assertFalse(old.exists())


class HandoverTests(unittest.TestCase):
    def test_command_passes_target_and_pid(self):
        command = updater.setup_command(Path("/t/Setup.exe"), Path("/apps/GPS Desktop"), 42)
        self.assertEqual(
            command[1:],
            ["--update", "--target", str(Path("/apps/GPS Desktop")), "--wait-pid", "42"],
        )

    def test_macos_bundle_is_opened_with_arguments(self):
        command = updater.setup_command(Path("/t/GPSDesktopSetup.app"), Path("/a/E.app"), 42)
        self.assertEqual(command[:4], ["open", "-n", str(Path("/t/GPSDesktopSetup.app")), "--args"])

    def test_source_runs_cannot_update(self):
        self.assertIsNone(updater.install_root())

    @unittest.skipIf(system.IS_MACOS, "macOS bundles do not use the marker")
    def test_frozen_copy_needs_the_install_marker(self):
        with tempfile.TemporaryDirectory() as temp:
            exe = Path(temp).resolve() / "GPSDesktop.exe"
            exe.write_bytes(b"")
            with mock.patch.object(sys, "frozen", True, create=True), mock.patch.object(
                sys, "executable", str(exe)
            ):
                self.assertIsNone(updater.install_root())
                (exe.parent / core.MARKER_NAME).write_text("{}")
                self.assertEqual(updater.install_root(), exe.parent)

    def test_wait_for_exit(self):
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        self.addCleanup(process.wait)
        self.addCleanup(process.kill)
        cancel = threading.Event()
        self.assertFalse(system.wait_for_exit(process.pid, cancel, timeout=0.3))

        cancel.set()
        with self.assertRaises(core.Cancelled):
            system.wait_for_exit(process.pid, cancel)
        cancel.clear()

        process.kill()
        process.wait()
        self.assertTrue(system.wait_for_exit(process.pid, cancel, timeout=10))

    def test_child_environment_drops_bundle_paths(self):
        with mock.patch.dict(os.environ, {"LD_LIBRARY_PATH": "/bundle", "QT_PLUGIN_PATH": "/bundle"}):
            env = system.child_environment()
        self.assertEqual(env["PYINSTALLER_RESET_ENVIRONMENT"], "1")
        self.assertNotIn("LD_LIBRARY_PATH", env)
        self.assertNotIn("QT_PLUGIN_PATH", env)


class RenameRetryTests(ServerTest):
    def test_a_briefly_locked_folder_is_retried(self):
        archive = self.root / "a.tar.gz"
        archive.write_bytes(tar_bytes({"new.txt": b"new"}))
        target = self.root / "GPS Desktop"
        core.install_archive(archive, target, "1.0.0")

        real_rename = Path.rename
        calls = []

        def flaky(path, destination):
            calls.append(path)
            if path == target and len(calls) == 1:
                raise PermissionError("locked")
            return real_rename(path, destination)

        with mock.patch.object(Path, "rename", flaky), mock.patch.object(
            core, "RENAME_DELAY_SECONDS", 0
        ):
            core.install_archive(archive, target, "2.0.0")
        self.assertEqual(core.read_marker(target)["version"], "2.0.0")

    def test_a_folder_that_stays_locked_keeps_the_old_install(self):
        archive = self.root / "a.tar.gz"
        archive.write_bytes(tar_bytes({"new.txt": b"new"}))
        target = self.root / "GPS Desktop"
        core.install_archive(archive, target, "1.0.0")

        with mock.patch.object(Path, "rename", side_effect=PermissionError("locked")), mock.patch.object(
            core, "RENAME_DELAY_SECONDS", 0
        ):
            with self.assertRaisesRegex(core.InstallError, "running"):
                core.install_archive(archive, target, "2.0.0")
        self.assertEqual(core.read_marker(target)["version"], "1.0.0")


class WizardUpdateTests(ServerTest):
    """Runs the real wizard in update mode, hidden, with OS integration stubbed."""

    def run_wizard(self, target, until, pid=None, act=lambda wizard: None):
        try:
            import tkinter

            from gps_desktop_setup import wizard as wizard_module

            with mock.patch.object(tkinter.Tk, "deiconify", lambda self: None):
                window = wizard_module.Wizard(target, pid)
        except (ImportError, tkinter.TclError) as error:
            self.skipTest(f"No display for tkinter: {error}")
        window.withdraw()
        closed = []
        real_destroy = window.destroy
        self.addCleanup(self.free_wizard, [window])
        window.destroy = lambda: (closed.append(True), real_destroy())
        act(window)
        deadline = time.time() + 30
        while not closed and not until(window):
            window.update()
            time.sleep(0.01)
            self.assertLess(time.time(), deadline, "wizard timed out")
        return window, closed

    def free_wizard(self, holder):
        """Destroy the wizard and free its Tk objects now, on the main thread.

        Left to the garbage collector, they can be freed later on a worker thread (such as the
        test HTTP server's), and Tcl aborts the whole process when that happens (seen on macOS).
        """
        import gc
        import tkinter

        window = holder.pop()
        # Pending after() callbacks are held by Tcl itself, out of the garbage collector's
        # reach, and keep the wizard alive; they also print "invalid command name ..._poll".
        # (Done by hand: after_cancel() fails on some Tcl versions, where the script is a list.)
        try:
            for job in window.tk.splitlist(window.tk.call("after", "info")):
                script = window.tk.splitlist(window.tk.call("after", "info", job))[0]
                window.tk.call("after", "cancel", job)
                command = str(script[0]) if isinstance(script, tuple) else str(script)
                window.deletecommand(command)
        except tkinter.TclError:
            pass
        try:
            type(window).destroy(window)  # The class's own, not run_wizard's recording wrapper.
        except tkinter.TclError:
            pass  # The wizard already closed itself.
        del window
        gc.collect()
        leaked = [obj for obj in gc.get_objects() if isinstance(obj, (tkinter.Variable, tkinter.Tk))]
        self.assertEqual(leaked, [], "wizard Tk objects outlived the test")

    def installed(self, version="0.1.0"):
        target = self.root / "programs" / "GPS Desktop"
        archive = self.root / "first.tar.gz"
        archive.write_bytes(tar_bytes({"GPSDesktop.exe": b"old"}))
        core.install_archive(archive, target, version)
        return target

    def stubs(self):
        return (
            mock.patch.object(core, "manifest_url", lambda repo: self.url),
            mock.patch.object(core, "platform_key", lambda: "test-x64"),
            mock.patch.object(system, "integrate", mock.Mock()),
            mock.patch.object(system, "launch", mock.Mock()),
            mock.patch.object(system, "app_executable", lambda d: d / "GPSDesktop.exe"),
        )

    def test_update_installs_and_reopens_the_app(self):
        target = self.installed()
        self.publish("0.2.0")
        a, b, c, d, e = self.stubs()
        with a, b, c as integrate, d as launch, e:
            _, closed = self.run_wizard(target, lambda w: False)
        self.assertTrue(closed)
        self.assertEqual((target / "GPSDesktop.exe").read_bytes(), b"new")
        if system.supports_uninstaller():
            self.assertEqual(core.read_marker(target)["version"], "0.2.0")
        else:  # macOS: extra files inside an app bundle would break code signing.
            self.assertFalse((target / core.MARKER_NAME).exists())
        integrate.assert_called_once()
        launch.assert_called_once_with(target)

    def test_failed_update_keeps_the_old_version_and_offers_retry(self):
        target = self.installed()
        a, b, c, d, e = self.stubs()  # Nothing published, so the manifest is missing.
        with a, b, c, d as launch, e:
            window, closed = self.run_wizard(target, lambda w: not w.busy)
            window.update()
            self.assertFalse(closed)
            self.assertEqual(str(window.next_button["text"]), "Try again")
            window.destroy()
            launch.assert_not_called()
        self.assertEqual((target / "GPSDesktop.exe").read_bytes(), b"old")

    def test_cancel_reopens_the_old_version(self):
        target = self.installed()
        self.publish("0.2.0")
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
        self.addCleanup(process.wait)
        self.addCleanup(process.kill)
        a, b, c, d, e = self.stubs()
        with a, b, c, d as launch, e:
            _, closed = self.run_wizard(
                target, lambda w: False, pid=process.pid, act=lambda w: w.cancel.set()
            )
        self.assertTrue(closed)
        launch.assert_called_once_with(target)
        self.assertEqual((target / "GPSDesktop.exe").read_bytes(), b"old")


class PromptTests(ServerTest):
    """Runs the real Qt prompt logic offscreen."""

    @classmethod
    def setUpClass(cls):
        try:
            from PySide6.QtWidgets import QApplication
        except ImportError as error:
            raise unittest.SkipTest(f"PySide6 not installed: {error}")
        cls.app = QApplication.instance() or QApplication([])

    def controller(self):
        from PySide6.QtWidgets import QMainWindow

        from gps_desktop import update_ui

        self.window = QMainWindow()
        return update_ui, update_ui.UpdateController(self.window, self.root / "GPS Desktop")

    def pump(self, condition):
        deadline = time.time() + 15
        while not condition():
            self.app.processEvents()
            time.sleep(0.005)
            self.assertLess(time.time(), deadline, "timed out")

    def check(self, update_ui, controller, result, manual=False, button=None):
        """Run a check, clicking ``button`` on the real prompt if it appears."""
        from PySide6.QtCore import QTimer
        from PySide6.QtWidgets import QApplication

        prompts = []

        def click_prompt():
            box = QApplication.activeModalWidget()
            if box is None:
                return
            prompts.append(box.text())
            if button is None:
                box.reject()  # An unexpected prompt: close it so the assertions can report it.
            else:
                {b.text(): b for b in box.buttons()}[button].click()

        timer = QTimer()
        timer.timeout.connect(click_prompt)
        timer.start(10)
        with mock.patch.object(updater, "find_update", side_effect=[result]), mock.patch.object(
            updater, "remove_old_downloads"
        ), mock.patch.object(update_ui.QMessageBox, "information") as information, mock.patch.object(
            update_ui.QMessageBox, "warning"
        ) as warning, mock.patch.object(
            controller, "_start_download"
        ) as start:
            controller.check(manual)
            self.pump(lambda: not controller.busy)
        timer.stop()
        return prompts, information, warning, start

    def update(self):
        return updater.Update("0.2.0", core.Release("0.2.0", "http://x/s", "0", 1))

    def test_up_to_date_auto_check_is_silent(self):
        update_ui, controller = self.controller()
        prompts, information, warning, _ = self.check(update_ui, controller, None)
        self.assertEqual(prompts, [])
        information.assert_not_called()
        warning.assert_not_called()

    def test_up_to_date_manual_check_says_so(self):
        update_ui, controller = self.controller()
        _, information, _, _ = self.check(update_ui, controller, None, manual=True)
        information.assert_called_once()

    def test_failed_auto_check_is_silent(self):
        update_ui, controller = self.controller()
        error = core.InstallError("offline")
        _, information, warning, _ = self.check(update_ui, controller, error)
        information.assert_not_called()
        warning.assert_not_called()

    def test_failed_manual_check_reports_it(self):
        update_ui, controller = self.controller()
        _, _, warning, _ = self.check(update_ui, controller, core.InstallError("offline"), manual=True)
        warning.assert_called_once()

    def test_later_does_not_download(self):
        update_ui, controller = self.controller()
        prompts, _, _, start = self.check(update_ui, controller, self.update(), button="Later")
        self.assertEqual(prompts, ["GPS Desktop 0.2.0 is available."])
        start.assert_not_called()

    def test_install_now_downloads(self):
        update_ui, controller = self.controller()
        prompts, _, _, start = self.check(update_ui, controller, self.update(), button="Install now")
        self.assertEqual(prompts, ["GPS Desktop 0.2.0 is available."])
        start.assert_called_once()

    def test_download_hands_over_and_quits(self):
        update_ui, controller = self.controller()
        self.publish("0.2.0")
        update = updater.find_update("0.1.0", self.url, "test-x64")
        with mock.patch.object(updater, "start_setup") as start_setup, mock.patch.object(
            update_ui.QApplication, "quit"
        ) as quit_app, mock.patch.object(tempfile, "tempdir", str(self.root)):
            controller._start_download(update)
            self.pump(lambda: not controller.busy)
        start_setup.assert_called_once()
        self.assertEqual(start_setup.call_args.args[1], self.root / "GPS Desktop")
        quit_app.assert_called_once()

    def test_failed_download_keeps_the_app_running(self):
        update_ui, controller = self.controller()
        update = updater.Update("0.2.0", core.Release("0.2.0", f"{self.base}/gone", "0", 1))
        with mock.patch.object(update_ui.QMessageBox, "warning") as warning, mock.patch.object(
            update_ui.QApplication, "quit"
        ) as quit_app, mock.patch.object(tempfile, "tempdir", str(self.root)):
            controller._start_download(update)
            self.pump(lambda: not controller.busy)
        warning.assert_called_once()
        quit_app.assert_not_called()

    def test_deferred_check_signals_then_hands_over_the_prompt(self):
        update_ui, controller = self.controller()
        events, deferred = [], []
        controller.check_finished.connect(lambda: events.append("finished"))

        def defer(prompt):
            events.append("deferred")
            deferred.append(prompt)

        with mock.patch.object(updater, "find_update", return_value=self.update()), mock.patch.object(
            updater, "remove_old_downloads"
        ), mock.patch.object(update_ui.QMessageBox, "exec") as shown:
            controller.check(defer=defer)
            self.pump(lambda: deferred)
            self.assertEqual(events, ["finished", "deferred"])
            shown.assert_not_called()  # Nothing appears until the caller runs the prompt.
            deferred[0]()
            shown.assert_called_once()


class SplashTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from PySide6.QtWidgets import QApplication
        except ImportError as error:
            raise unittest.SkipTest(f"PySide6 not installed: {error}")
        cls.app = QApplication.instance() or QApplication([])

    def make(self):
        from PySide6.QtGui import QIcon
        from PySide6.QtWidgets import QMainWindow

        from gps_desktop import splash

        self.window = QMainWindow()
        icon = QIcon(str(ROOT / "app" / "gps_desktop" / "assets" / "icon.png"))
        return splash, splash.Splash(icon, self.window)

    def wait(self, ms):
        deadline = time.time() + ms / 1000
        while time.time() < deadline:
            self.app.processEvents()
            time.sleep(0.005)

    def test_waits_for_ready_and_the_minimum_time(self):
        splash, startup = self.make()
        actions = []
        with mock.patch.object(splash, "MIN_DURATION_MS", 50), mock.patch.object(
            splash, "MAX_DURATION_MS", 5000
        ):
            startup.show()
            startup.ready()
            startup.after_reveal(lambda: actions.append("prompt"))
            self.assertFalse(self.window.isVisible())  # Ready, but the minimum has not passed.
            self.wait(150)
        self.assertTrue(self.window.isVisible())
        self.assertFalse(startup.screen.isVisible())
        self.assertEqual(actions, ["prompt"])

    def test_stays_until_ready(self):
        splash, startup = self.make()
        with mock.patch.object(splash, "MIN_DURATION_MS", 20), mock.patch.object(
            splash, "MAX_DURATION_MS", 5000
        ):
            startup.show()
            self.wait(100)
            self.assertFalse(self.window.isVisible())
            startup.ready()
        self.assertTrue(self.window.isVisible())

    def test_gives_up_waiting_at_the_maximum(self):
        splash, startup = self.make()
        with mock.patch.object(splash, "MIN_DURATION_MS", 20), mock.patch.object(
            splash, "MAX_DURATION_MS", 80
        ):
            startup.show()
            self.wait(200)
        self.assertTrue(self.window.isVisible())
        actions = []
        startup.after_reveal(lambda: actions.append("late prompt"))
        self.assertEqual(actions, ["late prompt"])


class IconTests(unittest.TestCase):
    def test_every_icon_file_exists(self):
        for path in (
            "app/gps_desktop/assets/icon.png",
            "installer/gps_desktop_setup/assets/icon.png",
            "packaging/assets/icon.ico",
            "packaging/assets/icon.icns",
        ):
            self.assertTrue((ROOT / path).is_file(), path)

    def test_linux_menu_entry_uses_the_bundled_icon(self):
        entry = system._desktop_entry(Path("/home/me/GPS Desktop"))
        icon = next(line for line in entry.splitlines() if line.startswith("Icon="))
        self.assertTrue(icon.replace("\\", "/").endswith("_internal/gps_desktop/assets/icon.png"))


class ManifestBuildTests(unittest.TestCase):
    def test_manifest_lists_apps_and_setups_for_each_platform(self):
        sys.path.insert(0, str(ROOT / "packaging"))
        import build

        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            for name in (
                "GPSDesktop-1.2.3-windows-x64.tar.gz",
                "GPSDesktop-1.2.3-macos-arm64.tar.gz",
                "GPSDesktop-1.2.3-linux-x64.tar.gz",
                "GPSDesktopSetup-windows-x64.exe",
                "GPSDesktopSetup-macos-arm64.zip",
                "GPSDesktopSetup-linux-x64.tar.gz",
            ):
                (folder / name).write_bytes(name.encode())
            build.manifest("1.2.3", "owner/repo", folder)
            manifest = json.loads((folder / "manifest.json").read_text())
        keys = {"windows-x64", "macos-arm64", "linux-x64"}
        self.assertEqual(set(manifest["assets"]), keys)
        self.assertEqual(set(manifest["setups"]), keys)
        self.assertEqual(manifest["setups"]["macos-arm64"]["name"], "GPSDesktopSetup-macos-arm64.zip")
        self.assertEqual(manifest["base_url"], "https://github.com/owner/repo/releases/download/desktop-v1.2.3")


if __name__ == "__main__":
    unittest.main()
