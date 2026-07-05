"""Records raw frames to a candump-format log, so a live or replayed
session can be saved and later fed back through `ReplayCanSource`.

Deliberately writes the exact format `replay.parse_candump_line` reads
and that `candump -l` produces, so logs are interchangeable with real
`can-utils` captures on the bench.
"""

from __future__ import annotations

import time
from pathlib import Path

from gps_dashboard.raw_frame import RawFrame


class CandumpRecorder:
    def __init__(self, path: Path | str, interface_label: str = "gps_dashboard") -> None:
        self.path = Path(path)
        self._interface_label = interface_label
        self._file = self.path.open("a", encoding="utf-8")

    def write(self, frame: RawFrame) -> None:
        wall_ts = time.time()
        data_hex = frame.data.hex().upper()
        can_id = f"{frame.arbitration_id:03X}"
        self._file.write(f"({wall_ts:.6f}) {self._interface_label} {can_id}#{data_hex}\n")

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> "CandumpRecorder":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()
