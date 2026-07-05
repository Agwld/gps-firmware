"""Common interface every CAN source (live/replay/simulate) implements.

The ingest worker doesn't know or care which one it's holding - this is
what lets live capture, log replay and the synthetic simulator share a
single decode/UI-update pipeline.
"""

from __future__ import annotations

import abc
import threading
from collections.abc import Iterator

from gps_dashboard.raw_frame import RawFrame


class CanSource(abc.ABC):
    """A source of raw CAN frames."""

    @abc.abstractmethod
    def frames(self, stop: threading.Event) -> Iterator[RawFrame]:
        """Yield frames until `stop` is set.

        Implementations must re-check `stop` at least every ~0.2 s (even
        while idle/blocked) so the UI's "disconnect" action is prompt,
        and must release any OS/hardware resources (bus handles, open
        files) before returning - including on the exception path.
        """
        raise NotImplementedError

    def send(self, frame: RawFrame) -> None:
        """Transmit a frame (e.g. a GPS_Command from the dashboard's
        control panel). No-op by default - a replayed log can't be
        written back to, and not every source is bidirectional. Live and
        simulate override this. Safe to call from the GUI thread while
        frames() runs in the ingest thread.
        """
        return None
