"""GPS data tests: the CAN database, simulator, data store, logs and the link. No hardware needed."""

import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "installer"), str(ROOT / "app")]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from gps_desktop.gps import enums  # noqa: E402
from gps_desktop.gps.dbc import COMMAND_MESSAGE, Database, Frame  # noqa: E402
from gps_desktop.gps.link import Link  # noqa: E402
from gps_desktop.gps.simulator import TICK, Simulator  # noqa: E402
from gps_desktop.gps.sources import Recorder, ReplaySource, parse_log_line  # noqa: E402
from gps_desktop.gps.store import GpsData, from_enu, to_enu  # noqa: E402

APP = QApplication.instance() or QApplication([])
DB = Database()


def run_simulator(simulator: Simulator, seconds: float, start: int = 0) -> list[Frame]:
    ticks = range(start, start + round(seconds / TICK))
    return [frame for tick in ticks for frame in simulator.step(tick, 1000.0 + tick * TICK)]


def fill(data: GpsData, frames: list[Frame]) -> None:
    data.set_connected(True, "test")
    data.ingest([(*DB.decode(frame), frame.t) for frame in frames])


class DatabaseTests(unittest.TestCase):
    def test_every_message_but_the_command_is_listed(self):
        names = [m.name for m in DB.messages]
        self.assertIn("GPS_Position", names)
        self.assertNotIn(COMMAND_MESSAGE, names)
        self.assertEqual(len(DB.signals), sum(len(m.signals) for m in DB.messages))

    def test_rates_come_from_the_comments(self):
        rates = {m.name: m.rate_hz for m in DB.messages}
        self.assertEqual(rates["GPS_IMU_Accel"], 100)
        self.assertEqual(rates["GPS_Position"], 50)
        self.assertIsNone(rates["Lap_Event"])  # Sent only when something happens

    def test_round_trip(self):
        frame = DB.encode("GPS_Velocity", {"speed_mps": 12.34, "course_deg": 271.5, "alt_m": -3.2,
                                           "fix_type": 3, "num_sv": 12, "counter": 7})
        name, values = DB.decode(frame)
        self.assertEqual(name, "GPS_Velocity")
        self.assertAlmostEqual(values["speed_mps"], 12.34)
        self.assertAlmostEqual(values["alt_m"], -3.2)

    def test_unknown_and_truncated_frames_are_rejected(self):
        self.assertIsNone(DB.decode(Frame(0x123, b"\x00" * 8)))
        position = next(m for m in DB.messages if m.name == "GPS_Position")
        self.assertIsNone(DB.decode(Frame(position.frame_id, b"\x00\x01")))

    def test_formatting_follows_the_scale(self):
        self.assertEqual(DB.signals["GPS_Position.lat_deg"].format(50.5), "50.5000000 deg")
        self.assertEqual(DB.signals["GPS_Velocity.num_sv"].format(9), "9")
        self.assertEqual(DB.signals["GPS_Velocity.num_sv"].format(None), "—")


class SimulatorTests(unittest.TestCase):
    def test_sends_every_message_at_its_rate(self):
        frames = run_simulator(Simulator(DB, seed=1), 10)
        counts: dict[str, int] = {}
        for frame in frames:
            decoded = DB.decode(frame)
            self.assertIsNotNone(decoded)
            counts[decoded[0]] = counts.get(decoded[0], 0) + 1
        for message in DB.messages:
            if message.rate_hz is not None:
                self.assertAlmostEqual(counts[message.name] / 10, message.rate_hz, delta=0.11, msg=message.name)

    def test_obeys_gate_commands(self):
        simulator = Simulator(DB, seed=1)
        data = GpsData(DB)

        def command(cmd: int, slot: int) -> None:
            simulator.send(DB.encode(COMMAND_MESSAGE, {"cmd": cmd, "arg0": slot, "arg1": 0}))

        command(enums.CMD_GATE_SET, enums.SLOT_NEXT_SECTOR)
        fill(data, run_simulator(simulator, 2))
        self.assertEqual(sorted(data.gates), [0, 1, 2, 3])
        command(enums.CMD_GATE_CLEAR, 2)
        fill(data, run_simulator(simulator, 2, start=200))
        self.assertEqual(sorted(data.gates), [0, 1, 3])
        command(enums.CMD_GATE_CLEAR, enums.SLOT_ALL)
        fill(data, run_simulator(simulator, 2, start=400))
        self.assertEqual(data.gates, {})
        self.assertFalse(data.flags("Lap_Status.flags") & enums.LAP_ARMED)


class StoreTests(unittest.TestCase):
    def test_laps_and_sectors_add_up(self):
        data = GpsData(DB)
        fill(data, run_simulator(Simulator(DB, lap_seconds=10, seed=1), 45))
        done = [lap for lap in data.laps if lap.time_ms is not None]
        self.assertGreaterEqual(len(done), 3)
        self.assertEqual([lap.number for lap in done], list(range(1, len(done) + 1)))
        for lap in done:
            self.assertEqual(len(lap.sectors), 3)
            self.assertAlmostEqual(sum(lap.sectors), lap.time_ms, delta=3)
        self.assertEqual(data.best_lap_ms, min(lap.time_ms for lap in done))

    def test_a_restarted_lap_count_starts_a_new_session(self):
        data = GpsData(DB)

        def event(kind: int, lap: int, time_ms: int, counter: int) -> None:
            data.ingest([("Lap_Event", {"type": kind, "lap": lap, "time_ms": time_ms, "counter": counter}, 0.0)])

        event(enums.EVENT_LAP, 1, 30000, 0)
        event(enums.EVENT_LAP, 2, 29000, 1)
        event(enums.EVENT_LAP, 2, 29000, 1)  # A repeat of the same event is ignored.
        self.assertEqual([lap.number for lap in data.laps], [1, 2])
        event(enums.EVENT_LAP, 1, 31000, 2)
        self.assertEqual([(lap.number, lap.time_ms) for lap in data.laps], [(1, 31000)])
        self.assertEqual(data.best_lap_ms, 31000)

    def test_track_and_origin(self):
        data = GpsData(DB)
        fill(data, run_simulator(Simulator(DB, seed=1), 5))
        self.assertIsNotNone(data.origin)
        east, north = data.trail_enu()
        self.assertEqual(len(east), 250)  # 50 Hz for 5 s
        self.assertLess(max(abs(east).max(), abs(north).max()), 200)

    def test_enu_round_trip(self):
        origin = (50.9357, -1.3966)
        lat, lon = from_enu(123.4, -56.7, origin)
        east, north = to_enu(lat, lon, origin)
        self.assertAlmostEqual(float(east), 123.4, places=6)
        self.assertAlmostEqual(float(north), -56.7, places=6)

    def test_stale_messages(self):
        data = GpsData(DB)
        self.assertEqual(data.stale, set(data.messages))  # Nothing is fresh while offline.
        data.set_connected(True, "test")
        data.ingest([("GPS_Position", {"lat_deg": 50.0, "lon_deg": -1.0}, time.monotonic())])
        self.assertFalse(data.is_stale("GPS_Position.lat_deg"))
        self.assertTrue(data.is_stale("GPS_Velocity.speed_mps"))
        self.assertNotIn("Lap_Event", data.stale)  # An event message has no rate to fall behind.
        data.reset()
        self.assertEqual(data.values, {})


class LogTests(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(parse_log_line("(1700000000.123456) can0 6B0#0102030405060708"),
                         (1700000000.123456, 0x6B0, bytes(range(1, 9))))
        self.assertIsNone(parse_log_line("# a comment"))
        self.assertIsNone(parse_log_line("(1.0) can0 6B0#012"))

    def test_a_recording_replays_identically(self):
        frames = run_simulator(Simulator(DB, seed=1), 1)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "session.log"
            recorder = Recorder(path)
            for frame in frames:
                recorder.write(frame)
            recorder.close()
            replayed = list(ReplaySource(path, speed=1000.0).frames(threading.Event()))
        self.assertEqual([(f.can_id, f.data) for f in replayed], [(f.can_id, f.data) for f in frames])

    def test_a_file_with_no_frames_is_an_error(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "notes.txt"
            path.write_text("not a log\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                list(ReplaySource(path).frames(threading.Event()))


class LinkTests(unittest.TestCase):
    def pump(self, seconds: float, until=lambda: False) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not until():
            APP.processEvents()
            time.sleep(0.005)

    def test_live_simulator_keeps_real_time(self):
        data = GpsData(DB)
        link = Link(Simulator(DB, seed=1), DB, data)
        link.start()
        self.pump(3.0)
        link.stop()
        self.assertAlmostEqual(data.counts["GPS_IMU_Accel"] / 3.0, 100, delta=8)
        self.assertEqual(data.unknown_frames, 0)

    def test_a_source_that_fails_reports_it(self):
        data = GpsData(DB)
        link = Link(ReplaySource("no-such-file.log"), DB, data)
        errors: list[str] = []
        link.failed.connect(errors.append)
        link.start()
        self.pump(3.0, until=lambda: bool(errors))
        link.stop()
        self.assertEqual(len(errors), 1)

    def test_a_finished_replay_says_so(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "session.log"
            recorder = Recorder(path)
            for frame in run_simulator(Simulator(DB, seed=1), 1):
                recorder.write(frame)
            recorder.close()
            data = GpsData(DB)
            link = Link(ReplaySource(path, speed=50.0), DB, data)
            ended: list[bool] = []
            link.ended.connect(lambda: ended.append(True))
            link.start()
            self.pump(5.0, until=lambda: bool(ended))
            link.stop()
        self.assertEqual(ended, [True])
        self.assertEqual(data.counts["GPS_IMU_Accel"], 100)


class WindowTests(unittest.TestCase):
    def test_every_screen_draws_from_simulated_data(self):
        from gps_desktop import theme
        from gps_desktop.ui.main_window import MainWindow

        theme.apply(APP)
        window = MainWindow()
        window._connect(lambda: None)
        fill(window.ctx.data, run_simulator(Simulator(DB, lap_seconds=10, seed=1), 25))
        for index, screen in enumerate(window.screens):
            window.show_screen(index)
            screen.refresh()
        dashboard = window.screens[0]
        self.assertEqual(dashboard.tiles["fix"].badge.text(), "3D")
        self.assertEqual(dashboard.tiles["rtk"].badge.text(), "Fixed")
        self.assertNotEqual(dashboard.tiles["lap_best"].value.text(), "—")
        window.ctx.disconnect_link()
        window.close()


class ReleaseTests(unittest.TestCase):
    def test_updates_come_from_the_fixed_app_tag_not_the_latest_release(self):
        from gps_desktop_setup import core

        self.assertEqual(core.manifest_url("owner/repo"),
                         "https://github.com/owner/repo/releases/download/desktop-latest/manifest.json")


if __name__ == "__main__":
    unittest.main()
