"""Map and satellite tiles: which tiles cover an area, fetching them, and caching them on disk.

Tiles are the usual 256-pixel Web Mercator ("slippy map") tiles. Each fetched tile is kept on
disk, so a track that has been viewed once still shows its map with no internet connection,
which is the normal state at a test day.
"""

from __future__ import annotations

import math
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QObject, Signal

TILE_PIXELS = 256
EARTH_CIRCUMFERENCE_M = 40075016.686
CACHE_DAYS = 30
TIMEOUT_SECONDS = 15
WORKERS = 4


@dataclass(frozen=True)
class Provider:
    key: str
    name: str
    url: str  # With {z}, {x} and {y}
    max_zoom: int
    attribution: str


PROVIDERS = {
    "map": Provider("map", "Map", "https://tile.openstreetmap.org/{z}/{x}/{y}.png", 19,
                    "Map data © OpenStreetMap contributors"),
    "satellite": Provider("satellite", "Satellite",
                          "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
                          19, "Imagery © Esri, Maxar, Earthstar Geographics and the GIS user community"),
}


def tile_xy(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Fractional tile coordinates of a point."""
    n = 2 ** zoom
    lat = max(-85.0511, min(85.0511, lat))
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def tile_corner(x: float, y: float, zoom: int) -> tuple[float, float]:
    """Latitude and longitude of a tile coordinate (a tile's top-left corner for whole numbers)."""
    n = 2 ** zoom
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat, lon


def zoom_for(metres_per_pixel: float, lat: float, max_zoom: int) -> int:
    """The zoom whose tiles are drawn closest to one tile pixel per screen pixel."""
    ground = EARTH_CIRCUMFERENCE_M * math.cos(math.radians(lat)) / TILE_PIXELS
    zoom = round(math.log2(ground / max(metres_per_pixel, 1e-3)))
    return max(1, min(max_zoom, zoom))


def tiles_covering(south: float, west: float, north: float, east: float, zoom: int) -> list[tuple[int, int]]:
    x0, y0 = tile_xy(north, west, zoom)
    x1, y1 = tile_xy(south, east, zoom)
    last = 2 ** zoom - 1
    xs = range(max(0, int(x0)), min(last, int(x1)) + 1)
    ys = range(max(0, int(y0)), min(last, int(y1)) + 1)
    return [(x, y) for y in ys for x in xs]


class TileCache(QObject):
    """Fetches tiles on background threads. `ready` brings back the image bytes on the GUI thread."""

    ready = Signal(str, int, int, int, bytes)  # provider key, zoom, x, y, image file bytes
    failed = Signal(str)  # A short message, at most once a minute

    def __init__(self, folder: Path, user_agent: str, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.folder = folder
        self.user_agent = user_agent
        self._pool = ThreadPoolExecutor(WORKERS, thread_name_prefix="tiles")
        self._pending: set[tuple] = set()
        self._missing: dict[tuple, float] = {}  # Tiles that failed, and when; not retried for a while
        self._lock = threading.Lock()
        self._last_failure_report = 0.0
        self.offline = False

    def path(self, provider: str, zoom: int, x: int, y: int) -> Path:
        return self.folder / provider / str(zoom) / str(x) / f"{y}.tile"

    def request(self, provider: str, zoom: int, x: int, y: int) -> None:
        key = (provider, zoom, x, y)
        with self._lock:
            if key in self._pending or time.monotonic() - self._missing.get(key, -1e9) < 60:
                return
            self._pending.add(key)
        self._pool.submit(self._load, key)

    def _load(self, key: tuple) -> None:
        provider, zoom, x, y = key
        path = self.path(*key)
        data = b""
        try:
            fresh = path.is_file() and time.time() - path.stat().st_mtime < CACHE_DAYS * 86400
            if fresh:
                data = path.read_bytes()
            else:
                data = self._fetch(PROVIDERS[provider], zoom, x, y)
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            self.offline = False
        except (OSError, urllib.error.URLError, TimeoutError) as error:
            if path.is_file():
                data = path.read_bytes()  # Out of date, but better than nothing while offline
            else:
                self._report(error)
        finally:
            with self._lock:
                self._pending.discard(key)
                if not data:
                    self._missing[key] = time.monotonic()
        if data:
            self.ready.emit(provider, zoom, x, y, data)

    def _fetch(self, provider: Provider, zoom: int, x: int, y: int) -> bytes:
        url = provider.url.format(z=zoom, x=x, y=y)
        request = urllib.request.Request(url, headers={"User-Agent": self.user_agent})
        with urllib.request.urlopen(request, timeout=TIMEOUT_SECONDS, context=_ssl_context()) as response:
            return response.read()

    def _report(self, error: Exception) -> None:
        self.offline = True
        now = time.monotonic()
        if now - self._last_failure_report > 60:
            self._last_failure_report = now
            self.failed.emit(f"Could not download map tiles ({error}). Areas viewed before still show.")

    def shutdown(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)


def _ssl_context():
    # Frozen builds cannot rely on the system's certificate store, so certifi is bundled.
    import ssl

    try:
        import certifi
    except ImportError:
        return ssl.create_default_context()
    return ssl.create_default_context(cafile=certifi.where())
