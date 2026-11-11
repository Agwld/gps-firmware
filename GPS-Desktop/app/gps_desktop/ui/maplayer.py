"""Map or satellite tiles drawn underneath a pyqtgraph plot whose axes are metres east and north."""

from __future__ import annotations

from collections import OrderedDict

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QObject, QRectF, QTimer
from PySide6.QtGui import QImage

from gps_desktop.gps import tiles
from gps_desktop.gps.store import from_enu, to_enu

MAX_TILES = 80  # Beyond this the view is too far out to be worth drawing at that zoom
MEMORY_TILES = 400  # Decoded tiles kept ready, so panning back is instant
BEHIND = -1000  # Tiles draw underneath everything else on the plot


def decode(data: bytes) -> np.ndarray | None:
    """Image file bytes as rows of RGBA pixels, top row first."""
    image = QImage.fromData(data)
    if image.isNull():
        return None
    image = image.convertToFormat(QImage.Format.Format_RGBA8888)
    width, height = image.width(), image.height()
    pixels = np.frombuffer(image.constBits(), np.uint8, count=image.sizeInBytes())
    return pixels.reshape(height, image.bytesPerLine() // 4, 4)[:, :width].copy()


class MapLayer(QObject):
    def __init__(self, plot: pg.PlotWidget, cache: tiles.TileCache) -> None:
        super().__init__(plot)
        self.plot = plot
        self.cache = cache
        self.provider: str | None = None
        self.origin: tuple[float, float] | None = None
        self.items: dict[tuple, pg.ImageItem] = {}
        self.images: OrderedDict[tuple, np.ndarray] = OrderedDict()
        self.wanted: set[tuple] = set()
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(120)  # Wait for a pan or zoom to settle before fetching
        self._timer.timeout.connect(self.update)
        plot.getViewBox().sigRangeChanged.connect(lambda *_: self._timer.start())
        cache.ready.connect(self._on_ready)

    def set_provider(self, key: str | None) -> None:
        if key == self.provider:
            return
        self.provider = key
        self._clear()
        self.update()

    def set_origin(self, origin: tuple[float, float] | None) -> None:
        if origin == self.origin:
            return
        self.origin = origin
        self._clear()  # Every tile moves relative to the new origin
        self.update()

    def attribution(self) -> str:
        return tiles.PROVIDERS[self.provider].attribution if self.provider else ""

    def _clear(self) -> None:
        for item in self.items.values():
            self.plot.removeItem(item)
        self.items.clear()
        self.wanted.clear()

    def update(self) -> None:
        if self.provider is None or self.origin is None:
            self._clear()
            return
        provider = tiles.PROVIDERS[self.provider]
        box = self.plot.getViewBox()
        (x0, x1), (y0, y1) = box.viewRange()
        pixels = max(1.0, box.width() * self.plot.devicePixelRatioF())
        south, west = from_enu(x0, y0, self.origin)
        north, east = from_enu(x1, y1, self.origin)
        zoom = tiles.zoom_for((x1 - x0) / pixels, self.origin[0], provider.max_zoom)
        covering = tiles.tiles_covering(south, west, north, east, zoom)
        while len(covering) > MAX_TILES and zoom > 1:
            zoom -= 1
            covering = tiles.tiles_covering(south, west, north, east, zoom)
        self.wanted = {(self.provider, zoom, x, y) for x, y in covering}
        for key in self.wanted:
            if key in self.images:
                self._show(key)
            else:
                self.cache.request(*key)
        self._tidy()

    def _on_ready(self, provider: str, zoom: int, x: int, y: int, data: bytes) -> None:
        key = (provider, zoom, x, y)
        pixels = decode(data)
        if pixels is None:
            return
        self.images[key] = pixels
        while len(self.images) > MEMORY_TILES:
            self.images.popitem(last=False)
        if key in self.wanted:
            self._show(key)
            self._tidy()

    def _show(self, key: tuple) -> None:
        self.images.move_to_end(key)
        if key in self.items or self.origin is None:
            return
        _provider, zoom, x, y = key
        north, west = tiles.tile_corner(x, y, zoom)
        south, east = tiles.tile_corner(x + 1, y + 1, zoom)
        left, top = to_enu(north, west, self.origin)
        right, bottom = to_enu(south, east, self.origin)
        # Image rows run north to south; the plot's y axis runs upwards, so flip them.
        item = pg.ImageItem(np.flipud(self.images[key]), axisOrder="row-major")
        item.setRect(QRectF(float(left), float(bottom), float(right - left), float(top - bottom)))
        item.setZValue(BEHIND + zoom)  # A sharper tile covers a blurrier one
        self.plot.addItem(item)
        self.items[key] = item

    def _tidy(self) -> None:
        """Drop tiles from other zooms once the ones wanted have all arrived, so the map never blanks."""
        if not all(key in self.items for key in self.wanted):
            return
        for key in [k for k in self.items if k not in self.wanted]:
            self.plot.removeItem(self.items.pop(key))
