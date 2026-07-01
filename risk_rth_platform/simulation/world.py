from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List

from .geometry import Point2D


@dataclass(frozen=True)
class NoFlyZone:
    """Circular no-fly zone used for simple 2D mission scenarios."""

    name: str
    center: Point2D
    radius_m: float

    def contains(self, point: Point2D) -> bool:
        return self.center.distance_to(point) <= self.radius_m


@dataclass(frozen=True)
class LandingSite:
    """Candidate emergency or diversion landing site."""

    name: str
    position: Point2D
    radius_m: float
    risk_score: float

    def contains(self, point: Point2D) -> bool:
        return self.position.distance_to(point) <= self.radius_m


@dataclass(frozen=True)
class SimulationWorld:
    """2D world definition for mission simulation."""

    width_m: float
    height_m: float
    home: Point2D
    no_fly_zones: List[NoFlyZone]
    landing_sites: List[LandingSite]

    def in_bounds(self, point: Point2D) -> bool:
        return 0.0 <= point.x <= self.width_m and 0.0 <= point.y <= self.height_m

    def violates_no_fly_zone(self, point: Point2D) -> bool:
        return any(zone.contains(point) for zone in self.no_fly_zones)

    def nearest_landing_site(self, point: Point2D) -> LandingSite | None:
        if not self.landing_sites:
            return None
        return min(
            self.landing_sites,
            key=lambda site: point.distance_to(site.position) * (1.0 + site.risk_score),
        )

    @classmethod
    def from_dict(cls, data: dict) -> "SimulationWorld":
        zones = [
            NoFlyZone(
                name=item["name"],
                center=Point2D(*item["center"]),
                radius_m=float(item["radius_m"]),
            )
            for item in data.get("no_fly_zones", [])
        ]
        sites = [
            LandingSite(
                name=item["name"],
                position=Point2D(*item["position"]),
                radius_m=float(item["radius_m"]),
                risk_score=float(item.get("risk_score", 0.2)),
            )
            for item in data.get("landing_sites", [])
        ]
        return cls(
            width_m=float(data["width_m"]),
            height_m=float(data["height_m"]),
            home=Point2D(*data["home"]),
            no_fly_zones=zones,
            landing_sites=sites,
        )
