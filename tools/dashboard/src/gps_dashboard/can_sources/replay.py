"""Replay a candump-format log file, pacing frames by their recorded
inter-frame timing (so a fast burst still looks like a fast burst).

Log line format (standard `candump -l`, one frame per line)::

    (1620000000.123456) can0 06B0#0102030405060708

Only the timestamp, ID and data fields are used; the interface name is
ignored (a replayed log can be "played back" regardless of which
interface it was originally captured on).
"""

from __future__ import annotations

import re
import threading
from collections.abc import Iterator
from pathlib import Path

from gps_dashboard.can_sources.base import CanSource
from gps_dashboard.raw_frame import RawFrame

_LINE_RE = re.compile(
    r"^\((?P<ts>\d+\.\d+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)\s*$"
)


def parse_candump_line(line: str) -> RawFrame | None:
    """Parse one candump-format line, or None if it doesn't match (blank
    lines, comments, extended/FD-frame lines with a second '#' etc. are
    all just skipped rather than raising)."""
    match = _LINE_RE.match(line.strip())
    if match is None:
        return None
    data_hex = match.group("data")
    if len(data_hex) % 2 != 0:
        return None
    return RawFrame(
        arbitration_id=int(match.group("id"), 16),
        data=bytes.fromhex(data_hex),
        timestamp=float(match.group("ts")),
    )


class ReplayCanSource(CanSource):
    def __init__(self, path: Path | str, speed: float = 1.0, loop: bool = True) -> None:
        """
        Args:
            path: candump-format log file.
            speed: Playback speed multiplier (2.0 = twice as fast, 0 =
                as fast as possible with no inter-frame delay).
            loop: Restart from the beginning at end-of-file.
        """
        self.path = Path(path)
        self.speed = speed
        self.loop = loop

    def frames(self, stop: threading.Event) -> Iterator[RawFrame]:
        while True:
            with self.path.open("r", encoding="utf-8", errors="replace") as f:
                prev_log_ts: float | None = None
                for line in f:
                    if stop.is_set():
                        return
                    frame = parse_candump_line(line)
                    if frame is None:
                        continue
                    if prev_log_ts is not None and self.speed > 0:
                        delay = (frame.timestamp - prev_log_ts) / self.speed
                        if delay > 0:
                            stop.wait(delay)
                    prev_log_ts = frame.timestamp
                    yield frame
            if not self.loop or stop.is_set():
                return
