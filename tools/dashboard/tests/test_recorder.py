from gps_dashboard.can_sources.replay import parse_candump_line
from gps_dashboard.raw_frame import RawFrame
from gps_dashboard.recorder import CandumpRecorder


def test_recorder_output_is_replayable(tmp_path):
    """The whole point of writing candump format is round-tripping
    through the replay parser without loss."""
    log = tmp_path / "recorded.log"
    frames = [
        RawFrame(arbitration_id=0x6B0, data=bytes.fromhex("0102030405060708"), timestamp=0.0),
        RawFrame(arbitration_id=0x6BF, data=bytes.fromhex("010203"), timestamp=0.0),
    ]

    with CandumpRecorder(log) as recorder:
        for frame in frames:
            recorder.write(frame)

    lines = log.read_text().splitlines()
    assert len(lines) == 2
    parsed = [parse_candump_line(line) for line in lines]
    assert [f.arbitration_id for f in parsed] == [0x6B0, 0x6BF]
    assert [f.data for f in parsed] == [f.data for f in frames]


def test_recorder_appends_across_instances(tmp_path):
    log = tmp_path / "recorded.log"
    with CandumpRecorder(log) as recorder:
        recorder.write(RawFrame(0x6B0, b"\x00", 0.0))
    with CandumpRecorder(log) as recorder:
        recorder.write(RawFrame(0x6B1, b"\x00", 0.0))

    assert len(log.read_text().splitlines()) == 2
