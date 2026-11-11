"""The app's update prompts and menu item."""

from __future__ import annotations

import threading
from collections.abc import Callable
from pathlib import Path

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox, QProgressDialog

from gps_desktop import __version__, updater
from gps_desktop_setup import core

class UpdateController(QObject):
    # Emitted when any check finishes, before any prompt is shown.
    check_finished = Signal()

    # Worker threads report back through signals, which Qt delivers on the GUI thread.
    _checked = Signal(object, bool)
    _check_failed = Signal(str, bool)
    _progress = Signal(int, int)
    _downloaded = Signal(object)
    _download_failed = Signal(str)
    _download_cancelled = Signal()

    def __init__(self, window: QMainWindow, target: Path) -> None:
        super().__init__(window)
        self.window = window
        self.target = target
        self.busy = False
        self.cancel = threading.Event()
        self.dialog: QProgressDialog | None = None
        self.defer: Callable[[Callable[[], None]], None] | None = None

        self._checked.connect(self._on_checked)
        self._check_failed.connect(self._on_check_failed)
        self._progress.connect(self._on_progress)
        self._downloaded.connect(self._on_downloaded)
        self._download_failed.connect(self._on_download_failed)
        self._download_cancelled.connect(self._close_dialog)

    def check(
        self, manual: bool = False, defer: Callable[[Callable[[], None]], None] | None = None
    ) -> None:
        """Look for a new version. Only a manual check reports "up to date" or failure.

        defer, if given, is handed the prompt to show and decides when to show it.
        """
        if self.busy:
            return
        self.busy = True
        self.defer = defer
        threading.Thread(target=self._check_worker, args=(manual,), daemon=True).start()

    def _check_worker(self, manual: bool) -> None:
        try:
            updater.remove_old_downloads()
            self._checked.emit(updater.find_update(__version__), manual)
        except Exception as error:  # A failed check must never disturb the app.
            self._check_failed.emit(str(error), manual)

    def _finish_check(self, prompt: Callable[[], None]) -> None:
        defer, self.defer = self.defer, None
        self.check_finished.emit()
        if defer is None:
            prompt()
        else:
            defer(prompt)

    def _on_check_failed(self, message: str, manual: bool) -> None:
        self.busy = False
        if manual:
            self._finish_check(lambda: QMessageBox.warning(self.window, "Check for Updates", message))
        else:
            self._finish_check(lambda: None)

    def _on_checked(self, update: updater.Update | None, manual: bool) -> None:
        self.busy = False
        self._finish_check(lambda: self._report(update, manual))

    def _report(self, update: updater.Update | None, manual: bool) -> None:
        if update is None:
            if manual:
                QMessageBox.information(
                    self.window,
                    "Check for Updates",
                    f"You have the latest version of {core.APP_NAME} ({__version__}).",
                )
            return

        prompt = QMessageBox(self.window)
        prompt.setIcon(QMessageBox.Icon.Information)
        prompt.setWindowTitle("Update Available")
        prompt.setText(f"{core.APP_NAME} {update.version} is available.")
        prompt.setInformativeText(
            f"You have version {__version__}. Installing the update closes {core.APP_NAME} "
            "and reopens it when finished."
        )
        install = prompt.addButton("Install now", QMessageBox.ButtonRole.AcceptRole)
        prompt.addButton("Later", QMessageBox.ButtonRole.RejectRole)
        prompt.exec()
        if prompt.clickedButton() is install:
            self._start_download(update)

    def _start_download(self, update: updater.Update) -> None:
        self.busy = True
        self.cancel.clear()
        self.dialog = QProgressDialog(
            f"Downloading {core.APP_NAME} {update.version}...",
            "Cancel",
            0,
            max(update.setup.size // 1024, 1),
            self.window,
        )
        self.dialog.setWindowTitle("Updating")
        self.dialog.setWindowModality(Qt.WindowModality.WindowModal)
        self.dialog.setMinimumDuration(0)
        self.dialog.setAutoClose(False)
        self.dialog.setAutoReset(False)
        self.dialog.canceled.connect(self.cancel.set)
        self.dialog.show()
        threading.Thread(target=self._download_worker, args=(update,), daemon=True).start()

    def _download_worker(self, update: updater.Update) -> None:
        try:
            program = updater.download_setup(
                update.setup,
                lambda done, total: self._progress.emit(done // 1024, total // 1024),
                self.cancel,
            )
            self._downloaded.emit(program)
        except core.Cancelled:
            self._download_cancelled.emit()
        except Exception as error:
            self._download_failed.emit(str(error))

    def _on_progress(self, done: int, _total: int) -> None:
        if self.dialog is not None:
            self.dialog.setValue(done)

    def _close_dialog(self) -> None:
        self.busy = False
        if self.dialog is not None:
            self.dialog.canceled.disconnect()
            self.dialog.close()
            self.dialog = None

    def _on_download_failed(self, message: str) -> None:
        self._close_dialog()
        QMessageBox.warning(self.window, "Update Failed", message)

    def _on_downloaded(self, program: Path) -> None:
        self._close_dialog()
        try:
            updater.start_setup(program, self.target)
        except OSError as error:
            QMessageBox.warning(self.window, "Update Failed", f"Could not start the update: {error}")
            return
        QApplication.quit()


def install(window: QMainWindow) -> UpdateController | None:
    """Add the update menu item to the window, if this copy is an installed one.

    The caller runs the startup check, so it can keep the splash screen up until it is done.
    """
    target = updater.install_root()
    if target is None:
        return None
    controller = UpdateController(window, target)
    help_menu = window.menuBar().addMenu("&Help")
    help_menu.addAction("Check for &Updates...", lambda: controller.check(manual=True))
    return controller
