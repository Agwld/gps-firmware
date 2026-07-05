from gps_dashboard.track_memory import TrackStore, gate_fingerprint

# A gate layout, as ENU metres relative to the node origin (what GPS_Gate
# carries). Index 0 = start/finish.
_GATES = {
    0: {"east_m": 0.0, "north_m": 0.0, "heading_deg": 90.0},
    1: {"east_m": 26.0, "north_m": 45.0, "heading_deg": 120.0},
    2: {"east_m": -26.0, "north_m": 45.0, "heading_deg": 240.0},
}


def test_fingerprint_is_none_without_gates():
    assert gate_fingerprint(50.0, -1.0, {}) is None


def test_fingerprint_stable_and_origin_independent():
    """Same physical gates reproduce the same id even when the node's ENU
    origin lands a little differently next boot: the gates' ENU are then
    reported relative to that new origin, but their absolute positions
    (origin + ENU) are unchanged, so the fingerprint must match."""
    fp1 = gate_fingerprint(50.0, -1.0, _GATES)

    # Shift the origin 10 m north and report every gate's ENU shifted the
    # opposite way, i.e. identical absolute gate positions.
    shifted_origin_lat = 50.0 + 10.0 / 111_320.0
    shifted = {
        i: {**g, "north_m": g["north_m"] - 10.0} for i, g in _GATES.items()
    }
    fp2 = gate_fingerprint(shifted_origin_lat, -1.0, shifted)

    assert fp1 == fp2


def test_fingerprint_differs_for_different_layout():
    moved = {**_GATES, 1: {"east_m": 200.0, "north_m": 200.0, "heading_deg": 10.0}}
    assert gate_fingerprint(50.0, -1.0, _GATES) != gate_fingerprint(
        50.0, -1.0, moved
    )


def test_store_roundtrip(tmp_path):
    store = TrackStore(tmp_path)
    fp = gate_fingerprint(50.0, -1.0, _GATES)
    assert fp is not None

    points = [(50.0, -1.0), (50.0001, -1.0001), (50.0002, -0.9999)]
    assert store.load(fp) is None  # nothing saved yet
    store.save(fp, points)

    loaded = store.load(fp)
    assert loaded is not None
    assert len(loaded) == 3
    for (a_lat, a_lon), (b_lat, b_lon) in zip(loaded, points):
        assert abs(a_lat - b_lat) < 1e-9
        assert abs(a_lon - b_lon) < 1e-9


def test_store_load_unknown_returns_none(tmp_path):
    assert TrackStore(tmp_path).load("deadbeef") is None
