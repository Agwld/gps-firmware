"""End-to-end installer logic tests against a local HTTP server standing in for GitHub."""

import functools
import hashlib
import http.server
import io
import json
import sys
import tarfile
import tempfile
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "installer"))

from gps_desktop_setup import core  # noqa: E402


def make_archive(path: Path, files: dict) -> None:
    with tarfile.open(path, "w:gz") as tar:
        for name, content in files.items():
            info = tarfile.TarInfo(name)
            info.size = len(content)
            tar.addfile(info, io.BytesIO(content))


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


class InstallerTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.site = self.root / "site"
        self.site.mkdir()
        self.target = self.root / "programs" / "GPS Desktop"

        handler = functools.partial(Quiet, directory=str(self.site))
        server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        self.base = f"http://127.0.0.1:{server.server_address[1]}"

    def publish(self, files=None, version="1.0.0", sha256=None):
        name = f"GPSDesktop-{version}-test-x64.tar.gz"
        path = self.site / name
        make_archive(path, files or {"GPSDesktop.exe": b"app", "_internal/lib.dll": b"lib"})
        manifest = {
            "version": version,
            "base_url": self.base,
            "assets": {
                "test-x64": {
                    "name": name,
                    "sha256": sha256 or hashlib.sha256(path.read_bytes()).hexdigest(),
                    "size": path.stat().st_size,
                }
            },
        }
        (self.site / "manifest.json").write_text(json.dumps(manifest))
        return core.fetch_release(f"{self.base}/manifest.json", "test-x64")

    def install(self, release):
        archive = self.root / "download.tar.gz"
        core.download(release, archive)
        core.install_archive(archive, self.target, release.version)

    def test_fresh_install(self):
        self.install(self.publish())
        self.assertEqual((self.target / "GPSDesktop.exe").read_bytes(), b"app")
        self.assertEqual((self.target / "_internal" / "lib.dll").read_bytes(), b"lib")
        self.assertEqual(core.read_marker(self.target)["version"], "1.0.0")

    def test_update_replaces_the_previous_version(self):
        self.install(self.publish({"old.txt": b"old"}))
        self.install(self.publish({"new.txt": b"new"}, version="2.0.0"))
        self.assertFalse((self.target / "old.txt").exists())
        self.assertTrue((self.target / "new.txt").exists())
        self.assertEqual(core.read_marker(self.target)["version"], "2.0.0")
        self.assertEqual(sorted(p.name for p in self.target.parent.iterdir()), ["GPS Desktop"])

    def test_refuses_a_folder_holding_other_files(self):
        self.target.mkdir(parents=True)
        (self.target / "thesis.docx").write_bytes(b"precious")
        with self.assertRaises(core.InstallError):
            self.install(self.publish())
        self.assertEqual((self.target / "thesis.docx").read_bytes(), b"precious")

    def test_empty_folder_is_accepted(self):
        self.target.mkdir(parents=True)
        self.install(self.publish())
        self.assertTrue((self.target / "GPSDesktop.exe").exists())

    def test_checksum_mismatch_rejected_and_file_removed(self):
        release = self.publish(sha256="0" * 64)
        archive = self.root / "download.tar.gz"
        with self.assertRaisesRegex(core.InstallError, "checksum"):
            core.download(release, archive)
        self.assertFalse(archive.exists())

    def test_failed_update_keeps_the_existing_install(self):
        self.install(self.publish({"keep.txt": b"keep"}))
        bad = self.root / "bad.tar.gz"
        bad.write_bytes(b"this is not a tarball")
        with self.assertRaises(core.InstallError):
            core.install_archive(bad, self.target, "2.0.0")
        self.assertEqual((self.target / "keep.txt").read_bytes(), b"keep")
        self.assertEqual(sorted(p.name for p in self.target.parent.iterdir()), ["GPS Desktop"])

    def test_path_traversal_in_archive_is_blocked(self):
        with self.assertRaises(core.InstallError):
            self.install(self.publish({"../escaped.txt": b"evil"}))
        self.assertFalse((self.target.parent / "escaped.txt").exists())

    def test_cancel_stops_the_download(self):
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(core.Cancelled):
            core.download(self.publish(), self.root / "x.tar.gz", cancel=cancel)

    def test_missing_platform_names_what_is_available(self):
        self.publish()
        with self.assertRaisesRegex(core.InstallError, "test-x64"):
            core.fetch_release(f"{self.base}/manifest.json", "plan9-mips")

    def test_no_release_published(self):
        with self.assertRaisesRegex(core.InstallError, "No published release"):
            core.fetch_release(f"{self.base}/missing.json", "test-x64")

    def test_uninstaller_is_copied_into_the_install(self):
        release = self.publish()
        archive = self.root / "download.tar.gz"
        core.download(release, archive)
        source = self.root / "setup-binary"
        source.write_bytes(b"setup")
        core.install_archive(archive, self.target, release.version, source, "uninstall")
        self.assertEqual((self.target / "uninstall").read_bytes(), b"setup")

    def test_update_keeps_shortcuts_on_record(self):
        self.install(self.publish())
        core.add_shortcuts_to_marker(self.target, ["desktop.lnk"])
        self.install(self.publish(version="2.0.0"))
        core.add_shortcuts_to_marker(self.target, ["menu.lnk"])
        self.assertEqual(core.read_marker(self.target)["shortcuts"], ["desktop.lnk", "menu.lnk"])

    def test_macos_bundle_is_replaced_without_a_marker(self):
        archive = self.root / "bundle.tar.gz"
        make_archive(archive, {"Contents/MacOS/GPSDesktop": b"app"})
        bundle = self.root / "GPS Desktop.app"
        core.install_archive(archive, bundle, "1.0.0", write_marker=False)
        core.install_archive(archive, bundle, "2.0.0", write_marker=False)
        self.assertFalse((bundle / core.MARKER_NAME).exists())
        self.assertTrue((bundle / "Contents" / "MacOS" / "GPSDesktop").is_file())

    def test_platform_key_shape(self):
        system, _, arch = core.platform_key().partition("-")
        self.assertTrue(system and arch)


if __name__ == "__main__":
    unittest.main()
