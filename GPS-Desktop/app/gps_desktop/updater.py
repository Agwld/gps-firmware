"""Find a newer release and hand over to its setup program. Contains no GUI code."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import threading
import urllib.parse
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from gps_desktop_setup import GITHUB_REPO, core, system

_DOWNLOAD_PREFIX = "gps-desktop-update-"
_SETUP_NAME = "GPSDesktopSetup"


@dataclass(frozen=True)
class Update:
    version: str
    setup: core.Release


def parse_version(text: str) -> Optional[tuple]:
    match = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)", text)
    return tuple(int(part) for part in match.groups()) if match else None


def is_newer(candidate: str, current: str) -> bool:
    """Compare release versions. Anything unrecognised, like a dev build, is never newer."""
    new, old = parse_version(candidate), parse_version(current)
    return new is not None and old is not None and new > old


def install_root() -> Optional[Path]:
    """Return the folder this app was installed in, or None if it can't update itself."""
    if not getattr(sys, "frozen", False):
        return None
    executable = Path(sys.executable).resolve()
    if system.IS_MACOS:
        bundle = executable.parents[2]
        return bundle if bundle.suffix == ".app" else None
    return executable.parent if (executable.parent / core.MARKER_NAME).is_file() else None


def find_update(current: str, url: Optional[str] = None, key: Optional[str] = None) -> Optional[Update]:
    """Return details of a newer release, or None if this version is up to date."""
    url = url or core.manifest_url(GITHUB_REPO)
    release = core.fetch_release(url, key)
    if not is_newer(release.version, current):
        return None
    return Update(release.version, core.fetch_setup(url, key))


def download_setup(
    setup: core.Release,
    on_progress: Callable[[int, int], None] = lambda done, total: None,
    cancel: Optional[threading.Event] = None,
) -> Path:
    """Download and verify the setup program, returning something that can be started."""
    folder = Path(tempfile.mkdtemp(prefix=_DOWNLOAD_PREFIX))
    try:
        path = folder / urllib.parse.unquote(setup.url.rsplit("/", 1)[-1])
        core.download(setup, path, on_progress, cancel)
        if tarfile.is_tarfile(path):
            core.extract_archive(path, folder)
            return folder / _SETUP_NAME
        if path.suffix == ".zip":
            # ditto preserves the symlinks and permissions that a macOS bundle relies on.
            subprocess.run(["ditto", "-x", "-k", str(path), str(folder)], check=True)
            return folder / f"{_SETUP_NAME}.app"
        return path
    except BaseException:
        shutil.rmtree(folder, ignore_errors=True)
        raise


def setup_command(program: Path, target: Path, pid: int) -> list[str]:
    arguments = ["--update", "--target", str(target), "--wait-pid", str(pid)]
    if program.suffix == ".app":
        return ["open", "-n", str(program), "--args", *arguments]
    return [str(program), *arguments]


def start_setup(program: Path, target: Path) -> None:
    """Start the setup program, which waits for this process to exit before installing."""
    subprocess.Popen(
        setup_command(program, target, os.getpid()),
        # Not the install folder: Windows can't replace a program's working directory.
        cwd=tempfile.gettempdir(),
        env=system.child_environment(),
        start_new_session=not system.IS_WINDOWS,
    )


def remove_old_downloads() -> None:
    """A running setup program can't delete itself, so the next app launch tidies up."""
    for folder in Path(tempfile.gettempdir()).glob(f"{_DOWNLOAD_PREFIX}*"):
        shutil.rmtree(folder, ignore_errors=True)
