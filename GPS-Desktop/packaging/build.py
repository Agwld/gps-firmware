#!/usr/bin/env python3
"""Build release files.

    python packaging/build.py build --version 1.2.3 --repo owner/name
    python packaging/build.py manifest --version 1.2.3 --repo owner/name --dir release

``build`` creates, in ./release, this platform's app archive and setup program.
``manifest`` hashes the app and setup files from every platform into manifest.json.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACKAGING = ROOT / "packaging"
RELEASE = ROOT / "release"
WORK = ROOT / "build"
DIST = ROOT / "dist"
APP_INIT = ROOT / "app" / "gps_desktop" / "__init__.py"
SETUP_INIT = ROOT / "installer" / "gps_desktop_setup" / "__init__.py"
APP_NAME = "GPSDesktop"
SETUP_NAME = "GPSDesktopSetup"
# The CAN database is the firmware's own, bundled so the app decodes exactly what the node sends.
DBC = ROOT.parent / "tools" / "GPS.dbc"
TAG_PREFIX = "desktop-v"  # Keeps app releases apart from firmware releases in the same repository

sys.path.insert(0, str(ROOT / "installer"))
from gps_desktop_setup.core import MANIFEST_NAME, platform_key  # noqa: E402


def pyinstaller(name: str, entry: str, paths: str, *options: str) -> None:
    subprocess.run(
        [
            sys.executable, "-m", "PyInstaller",
            "--noconfirm", "--clean", "--windowed",
            "--name", name,
            "--paths", str(ROOT / paths),
            "--distpath", str(DIST),
            "--workpath", str(WORK),
            "--specpath", str(WORK),
            "--icon", str(PACKAGING / "assets" / ("icon.icns" if sys.platform == "darwin" else "icon.ico")),
            *options,
            str(PACKAGING / entry),
        ],
        check=True,
    )


def stamp(path: Path, pattern: str, replacement: str) -> None:
    text, count = re.subn(pattern, replacement, path.read_text(encoding="utf-8"), flags=re.M)
    if count != 1:
        raise SystemExit(f"Could not stamp {path}")
    path.write_text(text, encoding="utf-8")


def build(version: str, repo: str) -> None:
    key = platform_key()
    originals = {path: path.read_text(encoding="utf-8") for path in (APP_INIT, SETUP_INIT)}
    shutil.rmtree(RELEASE, ignore_errors=True)
    RELEASE.mkdir()
    try:
        stamp(APP_INIT, r"^__version__ = .*$", f'__version__ = "{version}"')
        stamp(SETUP_INIT, r"^GITHUB_REPO = .*$", f'GITHUB_REPO = "{repo}"')
        # The app reuses the installer's download and checksum code for updates.
        pyinstaller(
            APP_NAME, "entry_app.py", "app", "--onedir",
            "--add-data", f"{ROOT / 'app' / 'gps_desktop' / 'assets'}{os.pathsep}gps_desktop/assets",
            "--add-data", f"{DBC}{os.pathsep}gps_desktop/assets",
            "--paths", str(ROOT / "installer"),
            "--hidden-import", "certifi", "--collect-data", "certifi",
            # python-can loads its interface backends by name at run time.
            "--collect-submodules", "can", "--copy-metadata", "python-can",
        )
        # A single file everywhere except macOS, where the .app bundle is the unit.
        # --hidden-import because certifi is imported inside a function.
        pyinstaller(
            SETUP_NAME, "entry_setup.py", "installer",
            "--onedir" if key.startswith("macos") else "--onefile",
            "--hidden-import", "certifi", "--collect-data", "certifi",
            "--add-data",
            f"{ROOT / 'installer' / 'gps_desktop_setup' / 'assets'}{os.pathsep}gps_desktop_setup/assets",
            "--exclude-module", "PySide6",
        )
    finally:
        for path, text in originals.items():
            path.write_text(text, encoding="utf-8")

    smoke_test(key)
    package_app(version, key)
    package_setup(key)


def smoke_test(key: str) -> None:
    if key.startswith("windows"):
        app, setup = DIST / APP_NAME / f"{APP_NAME}.exe", DIST / f"{SETUP_NAME}.exe"
    elif key.startswith("macos"):
        app = DIST / f"{APP_NAME}.app" / "Contents" / "MacOS" / APP_NAME
        setup = DIST / f"{SETUP_NAME}.app" / "Contents" / "MacOS" / SETUP_NAME
    else:
        app, setup = DIST / APP_NAME / APP_NAME, DIST / SETUP_NAME
    environment = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
    for program in (app, setup):
        subprocess.run([str(program), "--smoke-test"], check=True, timeout=120, env=environment)


def package_app(version: str, key: str) -> None:
    archive = RELEASE / f"{APP_NAME}-{version}-{key}.tar.gz"
    with tarfile.open(archive, "w:gz") as tar:
        if key.startswith("macos"):
            # The extraction folder itself becomes the .app bundle.
            source = DIST / f"{APP_NAME}.app"
        else:
            source = DIST / APP_NAME
        for child in sorted(source.iterdir()):
            tar.add(child, arcname=child.name)


def package_setup(key: str) -> None:
    if key.startswith("windows"):
        shutil.copy2(DIST / f"{SETUP_NAME}.exe", RELEASE / f"{SETUP_NAME}-{key}.exe")
    elif key.startswith("macos"):
        # ditto preserves the symlinks and metadata that a macOS bundle relies on.
        subprocess.run(
            ["ditto", "-c", "-k", "--keepParent", str(DIST / f"{SETUP_NAME}.app"),
             str(RELEASE / f"{SETUP_NAME}-{key}.zip")],
            check=True,
        )
    else:
        # A tarball, because browsers strip the executable bit from a bare binary.
        with tarfile.open(RELEASE / f"{SETUP_NAME}-{key}.tar.gz", "w:gz") as tar:
            tar.add(DIST / SETUP_NAME, arcname=SETUP_NAME)


def collect(directory: Path, pattern: str) -> dict:
    """Describe each file matching ``pattern``, keyed by the platform it captures."""
    found = {}
    for path in sorted(directory.iterdir()):
        match = re.fullmatch(pattern, path.name)
        if match:
            found[match.group(1)] = {
                "name": path.name,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "size": path.stat().st_size,
            }
    return found


def manifest(version: str, repo: str, directory: Path) -> None:
    assets = collect(directory, rf"{APP_NAME}-{re.escape(version)}-(.+)\.tar\.gz")
    setups = collect(directory, rf"{SETUP_NAME}-(.+?)(?:\.exe|\.zip|\.tar\.gz)")
    if not assets or set(assets) != set(setups):
        raise SystemExit(f"Expected matching app and setup files in {directory}")
    document = {
        "version": version,
        "base_url": f"https://github.com/{repo}/releases/download/{TAG_PREFIX}{version}",
        "assets": assets,
        "setups": setups,
    }
    (directory / MANIFEST_NAME).write_text(json.dumps(document, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["build", "manifest"])
    parser.add_argument("--version", required=True)
    parser.add_argument("--repo", required=True)
    parser.add_argument("--dir", type=Path, default=RELEASE)
    args = parser.parse_args()
    if args.command == "build":
        build(args.version, args.repo)
    else:
        manifest(args.version, args.repo, args.dir)


if __name__ == "__main__":
    main()
