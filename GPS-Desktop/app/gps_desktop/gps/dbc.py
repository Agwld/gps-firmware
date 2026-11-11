"""Decode and encode CAN frames with the firmware's own database, tools/GPS.dbc.

Nothing about the wire format is written down here. Every message, signal, scale and unit comes
from the DBC, which the firmware's tests keep in step with what the node really sends, so a new
signal shows up in the app without a code change.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

import cantools
from cantools.database.errors import Error as CantoolsError

COMMAND_MESSAGE = "GPS_Command"  # The one message sent to the node rather than by it


@dataclass(frozen=True, slots=True)
class Frame:
    """One raw CAN frame. `t` is in time.monotonic() seconds."""

    can_id: int
    data: bytes
    t: float = 0.0


@dataclass(frozen=True)
class Signal:
    cid: str  # "Message.signal", the id used everywhere in the app
    message: str
    name: str
    unit: str
    comment: str
    decimals: int

    def format(self, value: float | None, unit: bool = True) -> str:
        if value is None:
            return "—"
        text = f"{value:.{self.decimals}f}"
        return f"{text} {self.unit}" if unit and self.unit else text


@dataclass(frozen=True)
class Message:
    name: str
    frame_id: int
    length: int
    comment: str
    rate_hz: float | None  # Nominal rate, or None for a message sent only when something happens
    signals: tuple[Signal, ...]


def dbc_path() -> Path:
    """The bundled database in a release build, or the repository's own when run from source."""
    bundled = Path(__file__).resolve().parent.parent / "assets" / "GPS.dbc"
    if bundled.is_file():
        return bundled
    return Path(__file__).resolve().parents[4] / "tools" / "GPS.dbc"


def _decimals(scale: float) -> int:
    return 0 if scale >= 1 else min(7, math.ceil(-math.log10(scale)))


def _rate(comment: str) -> float | None:
    match = re.search(r"(\d+(?:\.\d+)?) Hz", comment)
    return float(match.group(1)) if match else None


class Database:
    def __init__(self, path: Path | str | None = None) -> None:
        self.path = Path(path) if path else dbc_path()
        self._db = cantools.database.load_file(str(self.path))
        self.messages: list[Message] = []
        self.signals: dict[str, Signal] = {}
        self._names: dict[int, str] = {}
        self.counter_size: dict[str, int] = {}  # Message name -> how many values its rolling counter has
        for m in self._db.messages:
            comment = m.comment or ""
            signals = tuple(
                Signal(f"{m.name}.{s.name}", m.name, s.name, s.unit or "", s.comment or "", _decimals(float(s.scale)))
                for s in m.signals
            )
            self._names[m.frame_id] = m.name
            self.counter_size.update({m.name: 1 << s.length for s in m.signals if s.name == "counter"})
            if m.name == COMMAND_MESSAGE:
                continue
            self.messages.append(Message(m.name, m.frame_id, m.length, comment, _rate(comment), signals))
            self.signals.update({s.cid: s for s in signals})

    def decode(self, frame: Frame) -> tuple[str, dict[str, float]] | None:
        """The message name and its signal values, or None for a frame the database can't read."""
        name = self._names.get(frame.can_id)
        if name is None:
            return None
        try:
            values = self._db.decode_message(frame.can_id, frame.data, decode_choices=False, allow_truncated=False)
        except (CantoolsError, ValueError):
            return None
        return name, values

    def encode(self, name: str, values: dict[str, float], t: float = 0.0) -> Frame:
        message = self._db.get_message_by_name(name)
        return Frame(message.frame_id, bytes(message.encode(values)), t)
