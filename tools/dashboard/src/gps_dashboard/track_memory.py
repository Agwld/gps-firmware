"""Remembered track outlines, keyed by the node's gate layout.

The GPS node doesn't broadcast the track shape - that would be a heavy,
point-by-point stream, and the node's flash is far too small to hold it.
Instead the *dashboard* remembers outlines locally: it accumulates the
driven breadcrumb live, saves it on exit, and reloads it on startup so
the track is on screen immediately instead of blank until the first lap.

The right outline is chosen by fingerprinting the gate layout the node
broadcasts. Because gates now persist as absolute lat/lon and reproduce
across power cycles, the same physical track produces the same
fingerprint next day - so revisiting a track reloads its outline, a
different (or cleared) layout starts fresh, and returning to the original
matches again. No track identity has to be typed in.

This module is pure logic (no Qt), so it's unit-testable on its own.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

_M_PER_DEG_LAT = 111_320.0

#: Rounding of a gate's absolute position for the fingerprint. 1e-5 deg is
#: ~1.1 m - far coarser than the gates' ~cm reproducibility (so the same
#: track matches reliably across boots) yet fine enough to tell distinct
#: tracks apart.
_FINGERPRINT_DECIMALS = 5


def _default_store_dir() -> Path:
    """Per-user data dir for saved outlines (XDG on Linux)."""
    import os

    base = os.environ.get("XDG_DATA_HOME") or str(Path.home() / ".local" / "share")
    return Path(base) / "gps-dashboard" / "tracks"


def gate_fingerprint(
    node_origin_lat: float,
    node_origin_lon: float,
    gates: dict[int, dict[str, float]],
) -> str | None:
    """Stable id for a gate layout, or None if there are no gates to key
    on. Gates are given as ENU metres (relative to the node origin); they
    are lifted to absolute lat/lon so the id is independent of which ENU
    origin this boot happened to land on.
    """
    if not gates:
        return None

    m_per_deg_lon = _M_PER_DEG_LAT * _cos_deg(node_origin_lat)
    parts: list[str] = []
    for idx in sorted(gates):
        g = gates[idx]
        lat = node_origin_lat + g["north_m"] / _M_PER_DEG_LAT
        lon = node_origin_lon + g["east_m"] / m_per_deg_lon
        parts.append(
            f"{idx}:{round(lat, _FINGERPRINT_DECIMALS)}:"
            f"{round(lon, _FINGERPRINT_DECIMALS)}"
        )
    digest = hashlib.sha1("|".join(parts).encode()).hexdigest()
    return digest[:16]


def _cos_deg(deg: float) -> float:
    import math

    return math.cos(math.radians(deg))


class TrackStore:
    """Loads/saves track outlines (lists of absolute lat/lon points) keyed
    by gate fingerprint, one JSON file per track."""

    def __init__(self, directory: Path | str | None = None) -> None:
        self._dir = Path(directory) if directory is not None else _default_store_dir()

    def _path(self, fingerprint: str) -> Path:
        return self._dir / f"{fingerprint}.json"

    def load(self, fingerprint: str) -> list[tuple[float, float]] | None:
        """Return the saved outline as (lat, lon) points, or None if this
        track hasn't been seen before / the file is unreadable."""
        path = self._path(fingerprint)
        if not path.is_file():
            return None
        try:
            data = json.loads(path.read_text())
            return [(float(p[0]), float(p[1])) for p in data["points"]]
        except (ValueError, KeyError, OSError):
            return None

    def save(self, fingerprint: str, points: list[tuple[float, float]]) -> None:
        """Persist an outline (absolute lat/lon points). Stored as
        absolute coordinates so it can be re-projected into whatever local
        frame a later session picks."""
        if not points:
            return
        self._dir.mkdir(parents=True, exist_ok=True)
        self._path(fingerprint).write_text(
            json.dumps({"points": [[lat, lon] for lat, lon in points]})
        )
