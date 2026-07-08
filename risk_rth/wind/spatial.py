"""Spatial wind-field models."""

from __future__ import annotations

from dataclasses import dataclass

from risk_rth.wind.field import WindSample, WindVector


@dataclass(frozen=True)
class GridWindField:
    """A rectangular 2D wind grid with bilinear interpolation.

    ``u_grid`` and ``v_grid`` are indexed as ``grid[y_index][x_index]``. The grid
    origin is at ``(origin_x_m, origin_y_m)`` and adjacent cells are separated by
    ``resolution_m``.
    """

    u_grid: tuple[tuple[float, ...], ...]
    v_grid: tuple[tuple[float, ...], ...]
    resolution_m: float
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0

    def __post_init__(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if not self.u_grid or not self.v_grid:
            raise ValueError("wind grids must not be empty")
        if len(self.u_grid) != len(self.v_grid):
            raise ValueError("u_grid and v_grid must have the same height")
        width = len(self.u_grid[0])
        if width == 0:
            raise ValueError("wind grids must not contain empty rows")
        for u_row, v_row in zip(self.u_grid, self.v_grid, strict=True):
            if len(u_row) != width or len(v_row) != width:
                raise ValueError("wind grid rows must have consistent width")

    @property
    def width(self) -> int:
        return len(self.u_grid[0])

    @property
    def height(self) -> int:
        return len(self.u_grid)

    def sample(self, x_m: float, y_m: float, t_s: float) -> WindSample:
        grid_x = (x_m - self.origin_x_m) / self.resolution_m
        grid_y = (y_m - self.origin_y_m) / self.resolution_m
        vector = WindVector(
            u=self._bilinear(self.u_grid, grid_x, grid_y),
            v=self._bilinear(self.v_grid, grid_x, grid_y),
        )
        return WindSample(x_m=x_m, y_m=y_m, t_s=t_s, vector=vector)

    def _bilinear(self, grid: tuple[tuple[float, ...], ...], x: float, y: float) -> float:
        x = min(max(x, 0.0), self.width - 1.0)
        y = min(max(y, 0.0), self.height - 1.0)

        x0 = int(x)
        y0 = int(y)
        x1 = min(x0 + 1, self.width - 1)
        y1 = min(y0 + 1, self.height - 1)
        tx = x - x0
        ty = y - y0

        top = (1.0 - tx) * grid[y0][x0] + tx * grid[y0][x1]
        bottom = (1.0 - tx) * grid[y1][x0] + tx * grid[y1][x1]
        return (1.0 - ty) * top + ty * bottom
