"""Tests for race time, the lap in progress, map tiles, saved locations and placing gates by position.
No hardware or internet needed."""

import math
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "installer"), str(ROOT / "app")]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QStandardPaths  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from gps_desktop.gps import enums, tiles  # noqa: E402
from gps_desktop.gps.dbc import GATE_SET_MESSAGE, Database  # noqa: E402
from gps_desktop.gps.link import Link  # noqa: E402
from gps_desktop.gps.locations import Location, Locations, PlannedGate  # noqa: E402
from gps_desktop.gps.simulator import ORIGIN, TICK, Simulator, track_point  # noqa: E402
from gps_desktop.gps.store import GpsData, from_enu  # noqa: E402

QStandardPaths.setTestModeEnabled(True)
APP = QApplication.instance() or QApplication([])
DB = Database()


def run(simulator: Simulator, data: GpsData, seconds: float, start: int = 0) -> int:
    data.set_connected(True, "test")
    ticks = range(start, start + round(seconds / TICK))
    data.ingest([(*DB.decode(f), f.t) for tick in ticks for f in simulator.step(tick, 1000.0 + tick * TICK)])
    return start + len(ticks)


def lap_status(data: GpsData, lap: int, running: bool, running_ms: int = 0) -> None:
    flags = (enums.LAP_RUNNING if running else 0) | enums.LAP_ARMED
    data.ingest([("Lap_Status", {"lap": lap, "running_time_ms": running_ms, "sector": 0, "flags": flags}, 0.0)])


def lap_event(data: GpsData, kind: int, lap: int, time_ms: int, counter: int) -> None:
    data.ingest([("Lap_Event", {"type": kind, "lap": lap, "time_ms": time_ms, "counter": counter}, 0.0)])


class LapProgressTests(unittest.TestCase):
    def test_a_lap_shows_in_progress_as_soon_as_it_starts(self):
        data = GpsData(DB)
        lap_status(data, 0, running=False)
        self.assertEqual(data.laps, [])
        lap_status(data, 0, running=True, running_ms=100)  # Crossed start/finish, before any sector
        self.assertEqual([(lap.number, lap.time_ms) for lap in data.laps], [(1, None)])
        lap_event(data, enums.EVENT_SECTOR, 0, 10000, 0)
        lap_event(data, enums.EVENT_LAP, 1, 30000, 1)
        lap_status(data, 0, running=True)  # A status frame lagging behind the event adds nothing
        lap_status(data, 1, running=True)  # The next lap is in progress straight away
        self.assertEqual([(lap.number, lap.time_ms) for lap in data.laps], [(1, 30000), (2, None)])

    def test_clearing_the_gates_removes_the_lap_in_progress(self):
        data = GpsData(DB)
        lap_status(data, 0, running=True)
        lap_status(data, 0, running=False)
        self.assertEqual(data.laps, [])

    def test_a_reset_lap_count_starts_again(self):
        data = GpsData(DB)
        for n in range(1, 5):
            lap_event(data, enums.EVENT_LAP, n, 30000, n)
        lap_status(data, 0, running=True)
        self.assertEqual([(lap.number, lap.time_ms) for lap in data.laps], [(1, None)])

    def test_race_time_adds_up_the_laps(self):
        data = GpsData(DB)
        self.assertIsNone(data.race_ms())
        lap_status(data, 0, running=True, running_ms=5000)
        self.assertEqual(data.race_ms(), 5000)
        lap_event(data, enums.EVENT_LAP, 1, 30000, 0)
        lap_event(data, enums.EVENT_LAP, 2, 29000, 1)
        lap_status(data, 2, running=True, running_ms=1234)
        self.assertEqual(data.race_ms(), 60234)
        self.assertFalse(data.missed_laps)

    def test_joining_mid_race_is_flagged(self):
        data = GpsData(DB)
        lap_status(data, 6, running=True, running_ms=1000)
        self.assertTrue(data.missed_laps)

    def test_simulated_laps_are_in_progress_from_the_start(self):
        data = GpsData(DB)
        simulator = Simulator(DB, lap_seconds=10, seed=1)
        tick = 0
        while not data.running() and tick < 2000:
            tick = run(simulator, data, 0.1, tick)
        self.assertTrue(data.running())
        self.assertEqual(data.laps[-1].time_ms, None)
        self.assertEqual(data.laps[-1].sectors, [])


class TileTests(unittest.TestCase):
    def test_round_trip(self):
        x, y = tiles.tile_xy(50.9357, -1.3966, 17)
        lat, lon = tiles.tile_corner(x, y, 17)
        self.assertAlmostEqual(lat, 50.9357, places=6)
        self.assertAlmostEqual(lon, -1.3966, places=6)

    def test_known_tile(self):
        # The tile containing Southampton at zoom 10 (checked against openstreetmap.org).
        x, y = tiles.tile_xy(50.9097, -1.4044, 10)
        self.assertEqual((int(x), int(y)), (508, 343))

    def test_zoom_matches_the_view(self):
        self.assertEqual(tiles.zoom_for(0.3, 51.0, 19), 18)  # 0.31 m per pixel at zoom 18
        self.assertLess(tiles.zoom_for(50, 51.0, 19), 13)
        self.assertEqual(tiles.zoom_for(0.01, 51.0, 19), 19)  # Never past the deepest zoom

    def test_covering(self):
        south, west = from_enu(-100, -100, ORIGIN)
        north, east = from_enu(100, 100, ORIGIN)
        covering = tiles.tiles_covering(south, west, north, east, 17)
        self.assertTrue(1 <= len(covering) <= 16)

    def test_cached_tiles_are_used_offline(self):
        with tempfile.TemporaryDirectory() as temp:
            cache = tiles.TileCache(Path(temp), "test")
            path = cache.path("map", 17, 1, 2)
            path.parent.mkdir(parents=True)
            path.write_bytes(b"cached")
            got: list = []
            cache.ready.connect(lambda *args: got.append(args))
            cache.request("map", 17, 1, 2)
            deadline = time.monotonic() + 5
            while not got and time.monotonic() < deadline:
                APP.processEvents()
                time.sleep(0.01)
            cache.shutdown()
        self.assertEqual(got, [("map", 17, 1, 2, b"cached")])


class LocationTests(unittest.TestCase):
    def test_save_and_reload(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "locations.json"
            store = Locations(path)
            place = Location("Thruxton", 51.21, -1.60, lines=[[[51.21, -1.60], [51.211, -1.601]]])
            place.set_gate(PlannedGate(0, 51.21, -1.60, 90.0))
            place.set_gate(PlannedGate(2, 51.211, -1.60, 180.0))
            store.add(place)
            again = Locations(path).get("Thruxton")
            self.assertEqual(again.lines, place.lines)
            self.assertEqual([g.slot for g in again.gates], [0, 2])
            self.assertEqual(again.next_sector(), 1)

    def test_a_new_start_finish_clears_the_sectors(self):
        place = Location("Track", 0.0, 0.0)
        place.set_gate(PlannedGate(1, 0, 0, 0))
        place.set_gate(PlannedGate(0, 0, 0, 0))
        self.assertEqual([g.slot for g in place.gates], [0])

    def test_rename_remove_and_nearest(self):
        with tempfile.TemporaryDirectory() as temp:
            store = Locations(Path(temp) / "locations.json")
            store.add(Location("A", 50.0, -1.0))
            store.add(Location("B", 52.0, -1.0))
            store.rename("A", "Home")
            self.assertEqual(store.names(), ["B", "Home"])
            self.assertEqual(store.nearest(50.001, -1.0).name, "Home")
            self.assertIsNone(store.nearest(51.0, -1.0))
            store.remove("B")
            self.assertEqual(store.names(), ["Home"])

    def test_import_and_a_damaged_file(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            other = Locations(folder / "other.json")
            other.add(Location("Shared", 51.0, -1.0))
            path = folder / "locations.json"
            path.write_text("{ not json", encoding="utf-8")
            store = Locations(path)
            self.assertTrue(store.error)
            self.assertTrue((folder / "locations.damaged.json").is_file())
            self.assertEqual(store.import_file(folder / "other.json"), 1)
            self.assertEqual(Locations(path).names(), ["Shared"])


class GateByPositionTests(unittest.TestCase):
    def test_the_message_is_outgoing_only(self):
        self.assertIn(GATE_SET_MESSAGE, DB.outgoing)
        self.assertNotIn(GATE_SET_MESSAGE, [m.name for m in DB.messages])

    def test_simulator_places_and_times_a_gate_from_a_position(self):
        data = GpsData(DB)
        simulator = Simulator(DB, lap_seconds=10, seed=1)
        tick = run(simulator, data, 1)
        # A start/finish across the track a quarter of the way round, facing the way the car drives.
        angle = 0.5 + math.pi / 2
        east, north = track_point(angle)
        ahead = track_point(angle + 0.01)
        heading = math.degrees(math.atan2(ahead[1] - north, ahead[0] - east)) % 360
        simulator.send(DB.encode(GATE_SET_MESSAGE, {"set_index": 0, "set_flags": 0, "set_east_m": east,
                                                    "set_north_m": north, "set_heading_deg": heading}))
        run(simulator, data, 25, tick)
        self.assertEqual(sorted(data.gates), [0])  # Start/finish cleared the sectors
        gate = data.gates[0]
        self.assertAlmostEqual(gate.east, east, delta=0.06)
        self.assertAlmostEqual(gate.heading, heading, delta=0.06)
        self.assertGreaterEqual(len([lap for lap in data.laps if lap.time_ms]), 1)

    def test_a_backwards_gate_never_times_a_lap(self):
        data = GpsData(DB)
        simulator = Simulator(DB, lap_seconds=10, seed=1)
        east, north = track_point(1.0)
        ahead = track_point(1.01)
        backwards = round((math.degrees(math.atan2(ahead[1] - north, ahead[0] - east)) + 180) % 360, 1) % 360
        simulator.send(DB.encode(GATE_SET_MESSAGE, {"set_index": 0, "set_flags": 0, "set_east_m": east,
                                                    "set_north_m": north, "set_heading_deg": backwards}))
        run(simulator, data, 25)
        self.assertEqual([lap for lap in data.laps if lap.time_ms], [])


class LayoutUploadTests(unittest.TestCase):
    def pump(self, seconds: float, until=lambda: False) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not until():
            APP.processEvents()
            time.sleep(0.005)

    def test_send_a_layout_and_confirm_it(self):
        from gps_desktop.ui.context import AppContext

        with tempfile.TemporaryDirectory() as temp:
            ctx = AppContext(folder=Path(temp))
            ctx.connect_simulator()
            self.pump(6.0, until=lambda: ctx.node_origin() is not None and len(ctx.data.gates) == 3)
            self.assertIsNotNone(ctx.node_origin())

            place = Location("Sim", *ORIGIN)
            for slot, angle in ((0, 2.0), (1, 3.5), (2, 5.0)):
                east, north = track_point(angle)
                lat, lon = from_enu(east, north, ctx.node_origin())
                place.set_gate(PlannedGate(slot, lat, lon, 45.0 * slot))
            self.assertTrue(ctx.layout_differences(place))  # The simulator starts with other gates

            results: list = []
            ctx.uploadFinished.connect(lambda ok, text: results.append((ok, text)))
            self.assertTrue(ctx.send_layout(place))
            self.pump(8.0, until=lambda: bool(results))
            self.assertEqual(results and results[0][0], True, results)
            self.assertEqual(ctx.layout_differences(place), [])

            # And back again: the node's gates saved into a location.
            copy = Location("Copy", *ORIGIN)
            ctx.locations.add(copy)
            self.assertEqual(ctx.copy_node_gates(copy), 3)
            self.assertEqual(ctx.layout_differences(copy), [])
            ctx.disconnect_link()
            ctx.tiles.shutdown()

    def test_sending_needs_a_node_position(self):
        from gps_desktop.ui.context import AppContext

        with tempfile.TemporaryDirectory() as temp:
            ctx = AppContext(folder=Path(temp))
            self.assertFalse(ctx.send_layout(Location("X", 0, 0)))
            self.assertFalse(ctx.set_gate_at(0, 0, 0, 0))
            ctx.tiles.shutdown()


class TrackScreenTests(unittest.TestCase):
    def test_place_a_gate_and_draw_a_line_with_clicks(self):
        from gps_desktop import theme
        from gps_desktop.ui.main_window import MainWindow
        from gps_desktop.ui.screens.track import TrackScreen

        theme.apply(APP)
        window = MainWindow()
        track = next(s for s in window.screens if isinstance(s, TrackScreen))
        track.map.set_provider(None)  # No map: no downloads, and the saved choice is left alone
        place = Location("Test track", *ORIGIN)
        window.ctx.locations.items = [place]
        window.ctx.locations.path = Path(tempfile.mkdtemp()) / "locations.json"
        track._fill_locations()
        track._select("Test track")
        self.assertIs(track.location, place)

        class Click:
            def __init__(self, point, double=False):
                box = track.plot.getViewBox()
                self._pos = box.mapViewToScene(pg_point(*point))
                self._double = double

            def button(self):
                from PySide6.QtCore import Qt
                return Qt.MouseButton.LeftButton

            def scenePos(self):
                return self._pos

            def double(self):
                return self._double

            def accept(self):
                pass

        track._start_drawing()
        for point in ((0, 0), (20, 5), (40, 0)):
            track._clicked(Click(point))
        track._clicked(Click((40, 0), double=True))
        self.assertEqual(len(place.lines), 1)
        self.assertEqual(len(place.lines[0]), 3)

        track._start_gate(0)
        track._clicked(Click((10, 0)))
        track._clicked(Click((10, 10)))  # Driven north through it
        gate = place.gate(0)
        self.assertIsNotNone(gate)
        self.assertAlmostEqual(gate.heading, 90.0, delta=0.5)
        self.assertEqual(track.gate_list.count(), 1)
        window.close()


def pg_point(x: float, y: float):
    from PySide6.QtCore import QPointF
    return QPointF(x, y)


if __name__ == "__main__":
    unittest.main()
