"""Core wind-field primitives.

A wind field maps a spatial position and time to a wind vector. The coordinate
system is intentionally simple: ``x`` and ``y`` are horizontal map coordinates in
metres, and ``t`` is simulation time in seconds.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot
from typing import Protocol


@dataclass(frozen=True)
class WindVector:
    """Two-dimensional wind vector in metres per second."""

    u: float
    v: float

    @property
    def speed_mps(self) -> float:
        """Return wind speed magnitude in metres per second."""

        return hypot(self.u, self.v)

    @property
    def direction_rad(self) -> float:
        """Return vector direction angle in radians."""

        return atan2(self.v, self.u)

    def dot(self, x: float, y: float) -> float:
        """Return dot product with another 2D vector."""

        return self.u * x + self.v * y


@dataclass(frozen=True)
class WindSample:
    """Wind-field sample at a given position and time."""

    x_m: float
    y_m: float
    t_s: float
    vector: WindVector

    @property
    def speed_mps(self) -> float:
        """Return sampled wind speed magnitude."""

        return self.vector.speed_mps


class WindField(Protocol):
    """Protocol implemented by deterministic and stochastic wind fields."""

    def sample(self, x_m: float, y_m: float, t_s: float) -> WindSample:
        """Return a wind sample for position ``(x_m, y_m)`` and time ``t_s``."""
