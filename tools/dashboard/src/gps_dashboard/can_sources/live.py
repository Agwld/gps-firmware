"""Live CAN source via python-can, backend-agnostic.

python-can's `interface` string selects the backend at runtime -
"socketcan" for a Linux SocketCAN device (can0, vcan0, ...), "vector"
for a Vector XL-API device, "kvaser" for a Kvaser CANlib device, "pcan"
for a PEAK PCAN device, etc. This class doesn't hardcode any of that; it
just forwards whatever the user picked in the connection panel.
"""

from __future__ import annotations

import threading
from collections.abc import Iterator
from typing import Any

import can

from gps_dashboard.can_sources.base import CanSource
from gps_dashboard.raw_frame import RawFrame

#: Backends worth offering in the UI dropdown. python-can supports more
#: (see `can.interface.VALID_INTERFACES`); this list is just the ones
#: this team is known to use plus the universal Linux fallback.
KNOWN_INTERFACES = ["socketcan", "vector", "kvaser", "pcan", "ixxat"]

#: Bus bitrate the whole car runs at (see board_config.h / the CAN
#: matrix); offered as the default, not enforced.
DEFAULT_BITRATE = 1_000_000


class LiveCanSource(CanSource):
    def __init__(
        self,
        interface: str,
        channel: str,
        bitrate: int = DEFAULT_BITRATE,
        **extra_kwargs: Any,
    ) -> None:
        """
        Args:
            interface: python-can backend name, e.g. "socketcan", "vector",
                "kvaser".
            channel: Backend-specific channel identifier, e.g. "can0" for
                socketcan, "0" for the first Vector/Kvaser channel.
            bitrate: Arbitration bitrate in bit/s.
            extra_kwargs: Passed straight through to `can.interface.Bus`,
                for backend-specific options (e.g. Vector's `app_name`).
        """
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.extra_kwargs = extra_kwargs

    def frames(self, stop: threading.Event) -> Iterator[RawFrame]:
        bus = can.interface.Bus(
            interface=self.interface,
            channel=self.channel,
            bitrate=self.bitrate,
            **self.extra_kwargs,
        )
        try:
            while not stop.is_set():
                msg = bus.recv(timeout=0.2)
                if msg is None or msg.is_error_frame or msg.is_remote_frame:
                    continue
                yield RawFrame(
                    arbitration_id=msg.arbitration_id,
                    data=bytes(msg.data),
                    timestamp=msg.timestamp,
                )
        finally:
            bus.shutdown()
