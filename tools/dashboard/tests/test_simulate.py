import threading
import time

from gps_dashboard.can_sources.simulate import SimulateCanSource
from gps_dashboard.decoder import Decoder


def test_simulate_produces_all_message_types_and_decodes_cleanly():
    # Defaults (30 m radius, 20 s/lap): every message type - even the
    # slowest, 1 Hz ones - appears within one second regardless of lap
    # timing, so there's no need to (and, per the clamping test below,
    # good reason not to) drive an unrealistically short lap period here.
    decoder = Decoder()
    source = SimulateCanSource(decoder)
    stop = threading.Event()

    seen_names: set[str] = set()

    def consume():
        for raw in source.frames(stop):
            decoded = decoder.decode(raw)
            assert decoded is not None, "simulator must only emit DBC-valid frames"
            seen_names.add(decoded.name)
            if len(seen_names) >= 12 and time.monotonic() - start > 1.1:
                stop.set()

    start = time.monotonic()
    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=5.0)
    assert not t.is_alive(), "simulator never stopped"

    expected = {
        "GPS_Position",
        "GPS_Velocity",
        "GPS_Attitude",
        "Lap_Status",
        "GPS_Quality",
        "GPS_IMU_Accel",
        "GPS_IMU_Gyro",
        "GPS_Temp",
        "GPS_Status",
        "GPS_Mag",
        "GPS_Frame_Origin",
        "GPS_Gate",
    }
    assert seen_names == expected


def test_simulate_emits_origin_and_all_gate_slots():
    """The frame origin decodes to the configured origin, and the gate
    round-robin marks exactly the four default slots (start/finish + 3
    sectors) valid and the rest empty."""
    decoder = Decoder()
    source = SimulateCanSource(
        decoder, origin_lat_deg=50.0, origin_lon_deg=-1.0
    )
    stop = threading.Event()

    valid_gates: set[int] = set()
    invalid_gates: set[int] = set()
    origin_seen: dict[str, float] = {}

    def consume():
        for raw in source.frames(stop):
            decoded = decoder.decode(raw)
            if decoded is None:
                continue
            if decoded.name == "GPS_Frame_Origin":
                origin_seen["lat"] = decoded.signals["origin_lat_deg"]
                origin_seen["lon"] = decoded.signals["origin_lon_deg"]
            elif decoded.name == "GPS_Gate":
                idx = int(decoded.signals["gate_index"])
                if decoded.signals["gate_flags"] & 0x01:
                    valid_gates.add(idx)
                else:
                    invalid_gates.add(idx)
            # Stop once every slot 0..7 has been broadcast at least once.
            if valid_gates | invalid_gates >= set(range(8)) and origin_seen:
                stop.set()
                return

    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=5.0)
    assert not t.is_alive(), "never saw all gate slots + origin"
    assert valid_gates == {0, 1, 2, 3}
    assert invalid_gates == {4, 5, 6, 7}
    assert abs(origin_seen["lat"] - 50.0) < 1e-6
    assert abs(origin_seen["lon"] - (-1.0)) < 1e-6


def test_simulate_emits_lap_and_sector_events():
    """Driving the default gates produces Lap_Event frames: sector
    crossings during the lap and a lap-complete at the line, with the lap
    time close to the configured lap period."""
    decoder = Decoder()
    source = SimulateCanSource(decoder, lap_period_s=4.0)
    stop = threading.Event()

    lap_times: list[int] = []
    sector_count = [0]

    def consume():
        for raw in source.frames(stop):
            decoded = decoder.decode(raw)
            if decoded is None or decoded.name != "Lap_Event":
                continue
            if decoded.signals["type"] == 0:  # CAN_LAP_EVENT_LAP
                lap_times.append(int(decoded.signals["time_ms"]))
            elif decoded.signals["type"] == 1:  # CAN_LAP_EVENT_SECTOR
                sector_count[0] += 1
            if lap_times:
                stop.set()
                return

    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=15.0)
    assert not t.is_alive(), "no lap completed"
    # First completed lap should be ~lap_period (4 s); allow generous slack
    # for the simulated wall-clock pacing.
    assert 3000 < lap_times[0] < 6000, lap_times
    assert sector_count[0] >= 3, "expected the 3 sector gates to be crossed"


def test_simulate_gate_commands_clear_and_set():
    """A GPS_Command GATE_CLEAR (all) empties every slot; a subsequent
    GATE_SET 0 re-marks the start/finish valid - i.e. the control panel
    can drive the simulated node's gates."""
    decoder = Decoder()
    source = SimulateCanSource(decoder)
    stop = threading.Event()

    phase = ["clear"]
    saw_all_empty = [False]
    saw_sf_valid_again = [False]

    def consume():
        for raw in source.frames(stop):
            decoded = decoder.decode(raw)
            if decoded is None or decoded.name != "GPS_Gate":
                continue
            idx = int(decoded.signals["gate_index"])
            valid = bool(decoded.signals["gate_flags"] & 0x01)

            if phase[0] == "clear":
                source.send(decoder.encode("GPS_Command", {"cmd": 0x02, "arg0": 0xFF, "arg1": 0}))
                phase[0] = "wait_empty"
            elif phase[0] == "wait_empty":
                if idx == 0 and not valid:
                    saw_all_empty[0] = True
                    source.send(decoder.encode("GPS_Command", {"cmd": 0x01, "arg0": 0, "arg1": 0}))
                    phase[0] = "wait_set"
            elif phase[0] == "wait_set":
                if idx == 0 and valid:
                    saw_sf_valid_again[0] = True
                    stop.set()
                    return

    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=10.0)
    assert not t.is_alive(), "gate command round-trip never completed"
    assert saw_all_empty[0], "clear-all did not empty the start/finish slot"
    assert saw_sf_valid_again[0], "GATE_SET 0 did not re-mark start/finish valid"


def test_simulate_position_stays_near_origin_on_a_bounded_track():
    decoder = Decoder()
    source = SimulateCanSource(
        decoder, track_radius_m=30.0, lap_period_s=5.0, origin_lat_deg=50.0, origin_lon_deg=-1.0
    )
    stop = threading.Event()

    def consume():
        for raw in source.frames(stop):
            decoded = decoder.decode(raw)
            if decoded and decoded.name == "GPS_Position":
                # 30 m radius track: never more than ~1e-3 deg from origin.
                assert abs(decoded.signals["lat_deg"] - 50.0) < 0.001
                assert abs(decoded.signals["lon_deg"] - (-1.0)) < 0.001
                stop.set()
                return

    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=5.0)
    assert not t.is_alive()


def test_simulate_clamps_unrealistic_accel_instead_of_crashing():
    """A tight radius + short lap period derives a centripetal
    acceleration far beyond GPS_IMU_Accel's encodable i16 range (and the
    real hardware's +-16 g FS) - this must be clamped, not raise out of
    the generator and silently kill the ingest thread."""
    decoder = Decoder()
    source = SimulateCanSource(decoder, track_radius_m=30.0, lap_period_s=1.0)
    stop = threading.Event()

    def consume():
        for raw in source.frames(stop):
            decoded = decoder.decode(raw)
            assert decoded is not None
            if decoded.name == "GPS_IMU_Accel":
                assert abs(decoded.signals["ay_mg"]) <= 16_000.0
                stop.set()
                return

    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=5.0)
    assert not t.is_alive(), "unrealistic track parameters crashed the simulator"
