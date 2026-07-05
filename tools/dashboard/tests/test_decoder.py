import math

from gps_dashboard.decoder import DEFAULT_DBC_PATH, Decoder
from gps_dashboard.raw_frame import RawFrame


def test_dbc_path_resolves_to_real_file():
    assert DEFAULT_DBC_PATH.name == "GPS.dbc"
    assert DEFAULT_DBC_PATH.is_file()


def test_decodes_every_message_name_present():
    decoder = Decoder()
    names = set(decoder.message_names)
    assert names == {
        "GPS_Position",
        "GPS_Velocity",
        "GPS_Attitude",
        "Lap_Status",
        "Lap_Event",
        "GPS_Quality",
        "GPS_IMU_Accel",
        "GPS_IMU_Gyro",
        "GPS_Temp",
        "GPS_Status",
        "GPS_Mag",
        "GPS_Command",
    }


def test_encode_decode_round_trip_gps_position():
    decoder = Decoder()
    raw = decoder.encode("GPS_Position", {"lat_deg": 50.9368, "lon_deg": -1.4045})
    decoded = decoder.decode(raw)
    assert decoded is not None
    assert decoded.name == "GPS_Position"
    assert decoded.frame_id == 0x6B0
    assert math.isclose(decoded.signals["lat_deg"], 50.9368, abs_tol=1e-6)
    assert math.isclose(decoded.signals["lon_deg"], -1.4045, abs_tol=1e-6)


def test_unknown_arbitration_id_returns_none():
    decoder = Decoder()
    raw = RawFrame(arbitration_id=0x123, data=b"\x00" * 8, timestamp=0.0)
    assert decoder.decode(raw) is None


def test_truncated_payload_returns_none_not_raise():
    decoder = Decoder()
    # GPS_Position is declared as 8 bytes; feed it 2.
    raw = RawFrame(arbitration_id=0x6B0, data=b"\x01\x02", timestamp=0.0)
    assert decoder.decode(raw) is None
