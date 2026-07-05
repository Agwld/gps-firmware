import threading
import time

from gps_dashboard.can_sources.replay import ReplayCanSource, parse_candump_line


def test_parse_candump_line_basic():
    frame = parse_candump_line("(1620000000.123456) can0 06B0#0102030405060708")
    assert frame is not None
    assert frame.arbitration_id == 0x6B0
    assert frame.data == bytes.fromhex("0102030405060708")
    assert frame.timestamp == 1620000000.123456


def test_parse_candump_line_lowercase_hex_and_short_dlc():
    frame = parse_candump_line("(0.0) vcan0 6b5#aabbcc")
    assert frame is not None
    assert frame.arbitration_id == 0x6B5
    assert frame.data == bytes.fromhex("aabbcc")


def test_parse_candump_line_rejects_garbage():
    assert parse_candump_line("") is None
    assert parse_candump_line("# a comment") is None
    assert parse_candump_line("(1.0) can0 6B0#odd_length_hex_x") is None


def test_replay_yields_frames_in_order_and_stops(tmp_path):
    log = tmp_path / "session.log"
    log.write_text(
        "(100.000000) can0 6B0#0102030405060708\n"
        "(100.000000) can0 6B1#0102030405060708\n"
        "(100.000000) can0 6B2#0102030405060708\n"
    )
    source = ReplayCanSource(log, speed=0, loop=False)
    stop = threading.Event()
    ids = [f.arbitration_id for f in source.frames(stop)]
    assert ids == [0x6B0, 0x6B1, 0x6B2]


def test_replay_loops_until_stopped(tmp_path):
    log = tmp_path / "session.log"
    log.write_text("(100.000000) can0 6B0#0102030405060708\n")
    source = ReplayCanSource(log, speed=0, loop=True)
    stop = threading.Event()

    seen = []

    def consume():
        for frame in source.frames(stop):
            seen.append(frame)
            if len(seen) >= 5:
                stop.set()

    t = threading.Thread(target=consume)
    t.start()
    t.join(timeout=2.0)
    assert not t.is_alive(), "replay with loop=True never stopped"
    assert len(seen) >= 5


def test_replay_respects_speed_pacing(tmp_path):
    log = tmp_path / "session.log"
    log.write_text(
        "(0.000000) can0 6B0#0102030405060708\n"
        "(0.050000) can0 6B1#0102030405060708\n"
    )
    source = ReplayCanSource(log, speed=1.0, loop=False)
    stop = threading.Event()
    start = time.monotonic()
    list(source.frames(stop))
    elapsed = time.monotonic() - start
    assert elapsed >= 0.04, "replay should pace out the 50 ms gap, not fire instantly"
