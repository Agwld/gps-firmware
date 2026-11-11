"""Where CAN frames come from: a live adapter or a recorded log. The simulator is a third source.

Every source looks the same to the rest of the app, so live data, a replayed log and the
simulator all go through one decode path. Logs use the candump format of can-utils, which other
CAN tools read and write too.
"""

from __future__ import annotations

import re
import threading
import time
from collections.abc import Iterator
from pathlib import Path

from gps_desktop.gps.dbc import Frame

DEFAULT_BITRATE = 1_000_000  # The car's sensor bus
# Adapters worth looking for. Each needs its maker's driver installed; missing ones are skipped.
INTERFACES = ["pcan", "kvaser", "vector", "ixxat", "slcan", "gs_usb", "socketcan"]

_LOG_LINE = re.compile(r"\((\d+(?:\.\d+)?)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)\s*$")


class Source:
    """A source of raw frames. `name` is shown to the user while it is connected."""

    name = ""

    def frames(self, stop: threading.Event) -> Iterator[Frame]:
        """Yield frames until `stop` is set, checking it at least a few times a second."""
        raise NotImplementedError

    def send(self, frame: Frame) -> bool:
        """Transmit a frame to the node. Returns False from a source that can't."""
        return False


def detect_channels() -> list[tuple[str, str]]:
    """(interface, channel) for every CAN adapter python-can can find. Slow; call from a thread."""
    import can

    found = []
    for interface in INTERFACES:
        try:
            configs = can.detect_available_configs(interfaces=[interface])
        except Exception:  # A broken vendor driver must not hide the other adapters.
            continue
        found.extend((str(c["interface"]), str(c["channel"])) for c in configs)
    return found


class LiveSource(Source):
    def __init__(self, interface: str, channel: str, bitrate: int = DEFAULT_BITRATE) -> None:
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.name = f"{interface} {channel}"
        self._bus = None
        self._lock = threading.Lock()

    def frames(self, stop: threading.Event) -> Iterator[Frame]:
        import can

        # python-can wants numbered channels (Kvaser, Vector) as integers.
        channel = int(self.channel) if self.channel.isdigit() else self.channel
        bus = can.Bus(interface=self.interface, channel=channel, bitrate=self.bitrate)
        with self._lock:
            self._bus = bus
        try:
            while not stop.is_set():
                message = bus.recv(timeout=0.1)
                if message is None or message.is_error_frame or message.is_remote_frame:
                    continue
                yield Frame(message.arbitration_id, bytes(message.data), time.monotonic())
        finally:
            with self._lock:
                self._bus = None
            bus.shutdown()

    def send(self, frame: Frame) -> bool:
        import can

        with self._lock:
            if self._bus is None:
                return False
            self._bus.send(can.Message(arbitration_id=frame.can_id, data=frame.data, is_extended_id=False))
        return True


def parse_log_line(line: str) -> tuple[float, int, bytes] | None:
    """(timestamp, id, data) from one candump line, or None for anything else."""
    match = _LOG_LINE.match(line.strip())
    if match is None or len(match.group(3)) % 2:
        return None
    return float(match.group(1)), int(match.group(2), 16), bytes.fromhex(match.group(3))


class ReplaySource(Source):
    """Plays a log back at the pace it was recorded, or `speed` times faster."""

    def __init__(self, path: Path | str, speed: float = 1.0, loop: bool = False) -> None:
        self.path = Path(path)
        self.speed = speed
        self.loop = loop
        self.name = f"Replay of {self.path.name}"

    def frames(self, stop: threading.Event) -> Iterator[Frame]:
        while not stop.is_set():
            first = None
            start = time.monotonic()
            with open(self.path, encoding="utf-8", errors="replace") as log:
                for line in log:
                    parsed = parse_log_line(line)
                    if parsed is None:
                        continue
                    stamp, can_id, data = parsed
                    if first is None:
                        first = stamp
                    due = start + (stamp - first) / self.speed
                    while (wait := due - time.monotonic()) > 0:
                        if stop.wait(min(wait, 0.1)):
                            return
                    if stop.is_set():
                        return
                    yield Frame(can_id, data, time.monotonic())
            if first is None:
                raise ValueError(f"{self.path.name} has no CAN frames in candump format.")
            if not self.loop:
                return


class Recorder:
    """Writes every frame to a candump log, which ReplaySource plays back."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)
        self._file = open(self.path, "w", encoding="utf-8", newline="\n")
        self._offset = time.time() - time.monotonic()  # Frames are stamped on the monotonic clock.
        self.frames = 0

    def write(self, frame: Frame) -> None:
        self._file.write(f"({frame.t + self._offset:.6f}) can0 {frame.can_id:03X}#{frame.data.hex().upper()}\n")
        self.frames += 1

    def close(self) -> None:
        self._file.close()
