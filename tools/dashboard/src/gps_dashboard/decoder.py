"""DBC-driven decoding of raw CAN frames, via `tools/GPS.dbc`.

Deliberately thin: cantools already knows every signal's scale, offset,
sign and byte order from the DBC (which is itself kept in lockstep with
the firmware's can_pack.c by tests/test_can_pack.c), so nothing about
the wire format is duplicated here. If the DBC changes, this dashboard
picks it up automatically on next launch - no code change needed.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cantools
from cantools.database import Database
from cantools.database.errors import DecodeError

from gps_dashboard.raw_frame import RawFrame

#: Default DBC location: tools/GPS.dbc, one directory up from this
#: package's project root (tools/dashboard/).
DEFAULT_DBC_PATH = Path(__file__).resolve().parents[3] / "GPS.dbc"


@dataclass(frozen=True, slots=True)
class DecodedMessage:
    """One successfully-decoded CAN frame."""

    name: str
    frame_id: int
    timestamp: float
    signals: dict[str, Any]


class Decoder:
    """Wraps a loaded `cantools` database for one-shot frame decoding."""

    def __init__(self, dbc_path: Path | str = DEFAULT_DBC_PATH) -> None:
        self._db: Database = cantools.database.load_file(str(dbc_path))
        self._by_id = {m.frame_id: m for m in self._db.messages}

    @property
    def message_names(self) -> list[str]:
        """Names of every message defined in the DBC, in file order."""
        return [m.name for m in self._db.messages]

    def decode(self, raw: RawFrame) -> DecodedMessage | None:
        """Decode one frame, or return None if its ID isn't in the DBC
        or its payload doesn't match the DBC's declared length/layout
        (e.g. a truncated/corrupt frame) - never raises."""
        message = self._by_id.get(raw.arbitration_id)
        if message is None:
            return None
        try:
            signals = self._db.decode_message(
                raw.arbitration_id, raw.data, allow_truncated=False
            )
        except (DecodeError, ValueError):
            return None
        return DecodedMessage(
            name=message.name,
            frame_id=raw.arbitration_id,
            timestamp=raw.timestamp,
            signals=signals,
        )

    def encode(self, name: str, signals: dict[str, Any]) -> RawFrame:
        """Encode signals into a wire frame (used by the simulator, and
        available for anyone recording a golden fixture by hand)."""
        message = self._db.get_message_by_name(name)
        data = self._db.encode_message(name, signals)
        return RawFrame(
            arbitration_id=message.frame_id, data=data, timestamp=time.monotonic()
        )
