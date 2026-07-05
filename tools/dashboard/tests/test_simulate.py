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
    round-robin covers all four simulated slots (start/finish + 3
    sectors), each valid."""
    decoder = Decoder()
    source = SimulateCanSource(
        decoder, origin_lat_deg=50.0, origin_lon_deg=-1.0
    )
    stop = threading.Event()

    gate_indices: set[int] = set()
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
                assert decoded.signals["gate_flags"] & 0x01, "gate must be valid"
                gate_indices.add(int(decoded.signals["gate_index"]))
            if gate_indices >= {0, 1, 2, 3} and origin_seen:
                stop.set()
                return

    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=5.0)
    assert not t.is_alive(), "never saw all gate slots + origin"
    assert gate_indices == {0, 1, 2, 3}
    assert abs(origin_seen["lat"] - 50.0) < 1e-6
    assert abs(origin_seen["lon"] - (-1.0)) < 1e-6


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
