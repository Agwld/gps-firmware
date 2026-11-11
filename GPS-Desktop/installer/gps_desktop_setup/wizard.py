"""The tkinter wizard: welcome, location, progress, finished.

With ``update_target`` it instead runs a shorter flow on behalf of the running app:
wait for the app to close, install the latest release over it, then reopen it.
"""

from __future__ import annotations

import queue
import sys
import tempfile
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from . import GITHUB_REPO, core, system

POLL_MS = 50


def set_icon(window: tk.Tk) -> None:
    try:
        window.iconphoto(True, tk.PhotoImage(master=window, file=str(core.ICON_PATH)))
    except tk.TclError:
        pass  # A missing icon is no reason to stop an install.


class Wizard(tk.Tk):
    def __init__(self, update_target: Path | None = None, wait_pid: int | None = None) -> None:
        super().__init__()
        self.update_target = update_target
        self.wait_pid = wait_pid
        self.title(f"{core.APP_NAME} Update" if update_target else f"{core.APP_NAME} Setup")
        self.geometry("560x380")
        set_icon(self)
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self._on_cancel)

        self.release: core.Release | None = None
        self.install_dir = tk.StringVar(value=str(system.default_install_dir()))
        self.desktop_shortcut = tk.BooleanVar(value=False)
        self.launch_after = tk.BooleanVar(value=True)
        self.events: queue.Queue = queue.Queue()
        self.cancel = threading.Event()
        self.busy = False
        self.committing = False  # Files are being swapped; closing now would corrupt them.
        self.finished = False

        self.body = ttk.Frame(self, padding=24)
        self.body.pack(fill="both", expand=True)
        ttk.Separator(self).pack(fill="x")
        buttons = ttk.Frame(self, padding=12)
        buttons.pack(fill="x")
        self.cancel_button = ttk.Button(buttons, text="Cancel", command=self._on_cancel)
        self.cancel_button.pack(side="right")
        self.next_button = ttk.Button(buttons, text="Next")
        self.next_button.pack(side="right", padx=8)
        self.back_button = ttk.Button(buttons, text="Back")
        self.back_button.pack(side="right")

        self.after(POLL_MS, self._poll)
        if update_target:
            self._start_update()
        else:
            self._show_welcome()

    # ---- Page helpers ----------------------------------------------------

    def _page(self, heading: str) -> None:
        for child in self.body.winfo_children():
            child.destroy()
        ttk.Label(self.body, text=heading, font=("TkDefaultFont", 15, "bold")).pack(
            anchor="w", pady=(0, 12)
        )

    def _text(self, text: str) -> ttk.Label:
        label = ttk.Label(self.body, text=text, wraplength=510, justify="left")
        label.pack(anchor="w", pady=(0, 12))
        return label

    def _buttons(self, next_text="Next", next_command=None, back_command=None) -> None:
        self.next_button.configure(
            text=next_text,
            command=next_command or (lambda: None),
            state="normal" if next_command else "disabled",
        )
        self.back_button.configure(
            command=back_command or (lambda: None),
            state="normal" if back_command else "disabled",
        )

    def _run(self, work) -> None:
        """Run ``work`` on a thread; failures come back through the event queue."""

        def target() -> None:
            try:
                work()
            except core.Cancelled:
                self.events.put(("cancelled",))
            except core.InstallError as error:
                self.events.put(("error", str(error)))
            except Exception as error:  # Never leave the wizard stuck on a dead thread.
                self.events.put(("error", f"Unexpected error: {error}"))

        threading.Thread(target=target, daemon=True).start()

    def _poll(self) -> None:
        try:
            while True:
                name, *args = self.events.get_nowait()
                getattr(self, f"_on_{name}")(*args)
        except queue.Empty:
            pass
        self.after(POLL_MS, self._poll)

    # ---- Update ----------------------------------------------------------

    def _start_update(self) -> None:
        self._page(f"Updating {core.APP_NAME}")
        self.status = self._text(f"Waiting for {core.APP_NAME} to close...")
        self.bar = ttk.Progressbar(self.body)
        self.bar.pack(fill="x")
        self._buttons()
        self.cancel_button.configure(state="normal")
        self.cancel.clear()
        self.busy = True
        target, pid = self.update_target, self.wait_pid

        def work() -> None:
            if pid is not None:
                system.wait_for_exit(pid, self.cancel)
            release = core.fetch_release(core.manifest_url(GITHUB_REPO))
            self.events.put(("update_release", release))
            self._install(target, release, desktop=False)

        self._run(work)

    def _on_update_release(self, release: core.Release) -> None:
        self.release = release
        self.bar.configure(maximum=release.size)

    def _leave_without_installing(self) -> None:
        """Close the wizard. During an update, reopen the version the user was using."""
        if self.update_target and system.app_executable(self.update_target).exists():
            try:
                system.launch(self.update_target)
            except OSError:
                pass  # It can still be opened from the Start Menu or Applications.
        self.destroy()

    # ---- Welcome ---------------------------------------------------------

    def _show_welcome(self) -> None:
        self._page(f"Welcome to {core.APP_NAME} Setup")
        self._text(f"This wizard downloads and installs the latest version of {core.APP_NAME}.")
        self.status = self._text("Checking for the latest version...")
        self._buttons()
        self.cancel_button.configure(state="normal")
        self.cancel.clear()
        self.busy = True

        def work() -> None:
            release = core.fetch_release(core.manifest_url(GITHUB_REPO))
            self.events.put(("release", release))

        self._run(work)

    def _on_release(self, release: core.Release) -> None:
        self.busy = False
        if self.cancel.is_set():
            self._leave_without_installing()
            return
        self.release = release
        megabytes = release.size / 1_000_000
        self.status.configure(text=f"Version {release.version} ({megabytes:.0f} MB download).")
        self._buttons(next_command=self._show_location)

    # ---- Location --------------------------------------------------------

    def _show_location(self) -> None:
        self._page("Choose where to install")
        self._text(f"{core.APP_NAME} will be installed in this folder:")
        row = ttk.Frame(self.body)
        row.pack(fill="x", pady=(0, 12))
        ttk.Entry(row, textvariable=self.install_dir).pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Browse...", command=self._browse).pack(side="left", padx=(8, 0))
        if not system.IS_MACOS:
            ttk.Checkbutton(
                self.body, text="Create a desktop shortcut", variable=self.desktop_shortcut
            ).pack(anchor="w")
        self._buttons("Install", self._start_install, self._show_welcome)

    def _browse(self) -> None:
        chosen = filedialog.askdirectory(initialdir=str(Path(self.install_dir.get()).parent))
        if chosen:
            # Always use a dedicated subfolder, never the chosen folder itself.
            name = f"{core.APP_NAME}.app" if system.IS_MACOS else core.APP_NAME
            self.install_dir.set(str(Path(chosen) / name))

    # ---- Progress --------------------------------------------------------

    def _start_install(self) -> None:
        target = Path(self.install_dir.get()).expanduser()
        if system.IS_MACOS and target.suffix != ".app":
            target = target / f"{core.APP_NAME}.app"
        if not target.is_absolute():
            messagebox.showerror("Setup", "Please enter a full folder path.", parent=self)
            return
        try:
            core.check_target(target)
        except core.InstallError as error:
            messagebox.showerror("Setup", str(error), parent=self)
            return

        self._page(f"Installing {core.APP_NAME}")
        self.status = self._text("Downloading...")
        self.bar = ttk.Progressbar(self.body, maximum=self.release.size)
        self.bar.pack(fill="x")
        self._buttons()
        self.cancel.clear()
        self.busy = True
        # Read now: tkinter variables must not be touched from the worker thread.
        release, desktop = self.release, self.desktop_shortcut.get()
        self._run(lambda: self._install(target, release, desktop))

    def _install(self, target: Path, release: core.Release, desktop: bool) -> None:
        with tempfile.TemporaryDirectory(prefix="gps-desktop-setup-") as temp:
            archive = Path(temp) / "payload.tar.gz"
            core.download(
                release,
                archive,
                lambda done, total: self.events.put(("progress", done, total)),
                self.cancel,
            )
            if self.cancel.is_set():
                raise core.Cancelled()
            self.events.put(("status", "Installing..."))

            # A frozen wizard copies itself into place to act as the uninstaller.
            source = None
            if getattr(sys, "frozen", False) and system.supports_uninstaller():
                source = Path(sys.executable)
            core.install_archive(
                archive,
                target,
                release.version,
                source,
                system.UNINSTALLER_NAME,
                write_marker=system.supports_uninstaller(),
            )
            system.integrate(target, release.version, release.size, desktop)
        self.events.put(("installed", target))

    def _on_progress(self, done: int, total: int) -> None:
        self.bar.configure(value=done)
        self.status.configure(
            text=f"Downloading... {done / 1_000_000:.0f} of {total / 1_000_000:.0f} MB"
        )

    def _on_status(self, text: str) -> None:
        self.committing = True
        self.cancel_button.configure(state="disabled")
        self.status.configure(text=text)
        self.bar.configure(mode="indeterminate")
        self.bar.start()

    def _on_installed(self, target: Path) -> None:
        self.busy = False
        self.committing = False
        self.finished = True
        if self.update_target:
            self._finish(target)
            return
        self._page("Installation complete")
        self._text(f"{core.APP_NAME} {self.release.version} is installed.")
        ttk.Checkbutton(
            self.body, text=f"Open {core.APP_NAME} now", variable=self.launch_after
        ).pack(anchor="w")
        self._buttons("Finish", lambda: self._finish(target))
        self.cancel_button.configure(state="disabled")

    def _finish(self, target: Path) -> None:
        if self.launch_after.get():
            try:
                system.launch(target)
            except OSError as error:
                messagebox.showerror("Setup", f"Could not open the app: {error}", parent=self)
        self.destroy()

    # ---- Failure and cancel ----------------------------------------------

    def _on_error(self, message: str) -> None:
        self.busy = False
        self.committing = False
        if self.cancel.is_set():
            self._leave_without_installing()
            return
        self._page("Setup could not finish")
        self._text(message)
        self.cancel_button.configure(state="normal")
        self._buttons("Try again", self._start_update if self.update_target else self._show_welcome)

    def _on_cancelled(self) -> None:
        self._leave_without_installing()

    def _on_cancel(self) -> None:
        if self.committing:
            return
        if self.finished:
            self.destroy()
        elif messagebox.askyesno("Setup", f"Cancel {core.APP_NAME} Setup?", parent=self):
            self.cancel.set()
            if not self.busy:
                self._leave_without_installing()


def run_wizard() -> int:
    Wizard().mainloop()
    return 0


def run_update(target: Path, wait_pid: int | None) -> int:
    Wizard(target, wait_pid).mainloop()
    return 0


def run_uninstall(install_dir: Path) -> int:
    root = tk.Tk()
    root.withdraw()
    set_icon(root)
    title = f"Uninstall {core.APP_NAME}"
    if not messagebox.askyesno(title, f"Remove {core.APP_NAME} from {install_dir}?"):
        return 0
    try:
        system.uninstall(install_dir)
    except (core.InstallError, OSError) as error:
        messagebox.showerror(title, str(error))
        return 1
    messagebox.showinfo(title, f"{core.APP_NAME} has been removed.")
    system.finish_uninstall(install_dir)
    return 0
