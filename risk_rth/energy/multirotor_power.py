"""Transparent multirotor-inspired power model.

Research Prototype / Simulation-Only. The equations expose physically meaningful
components but are not platform-identified or flight validated.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class PowerBreakdown:
    induced_w: float
    profile_w: float
    parasitic_w: float
    climb_w: float
    avionics_w: float
    total_w: float
    standard_deviation_w: float

    def to_dict(self) -> dict[str, float]:
        return {key: float(value) for key, value in asdict(self).items()}


@dataclass(frozen=True)
class MultirotorPowerModel:
    """Component model based on momentum-theory and drag-inspired terms."""

    mass_kg: float
    rotor_disk_area_m2: float
    air_density_kg_m3: float = 1.225
    profile_power_w: float = 35.0
    parasitic_drag_coefficient: float = 0.08
    reference_area_m2: float = 0.12
    avionics_power_w: float = 12.0
    drivetrain_efficiency: float = 0.85
    descent_recovery_factor: float = 0.25
    relative_uncertainty: float = 0.10
    gravity_m_s2: float = 9.80665

    def __post_init__(self) -> None:
        positive = {
            "mass_kg": self.mass_kg,
            "rotor_disk_area_m2": self.rotor_disk_area_m2,
            "air_density_kg_m3": self.air_density_kg_m3,
            "drivetrain_efficiency": self.drivetrain_efficiency,
        }
        if any(value <= 0 for value in positive.values()):
            raise ValueError(f"positive parameters required: {positive}")
        if not 0 < self.drivetrain_efficiency <= 1:
            raise ValueError("drivetrain_efficiency must be in (0, 1]")
        if self.relative_uncertainty < 0:
            raise ValueError("relative_uncertainty must be non-negative")

    @property
    def hover_induced_power_w(self) -> float:
        thrust_n = self.mass_kg * self.gravity_m_s2
        ideal = thrust_n ** 1.5 / math.sqrt(
            2.0 * self.air_density_kg_m3 * self.rotor_disk_area_m2
        )
        return ideal / self.drivetrain_efficiency

    def estimate(
        self,
        airspeed_m_s: float,
        climb_rate_m_s: float = 0.0,
        payload_multiplier: float = 1.0,
        safety_factor: float = 1.0,
    ) -> PowerBreakdown:
        if airspeed_m_s < 0 or payload_multiplier <= 0 or safety_factor < 1:
            raise ValueError("invalid speed, payload multiplier, or safety factor")
        induced = self.hover_induced_power_w * payload_multiplier ** 1.5
        profile = self.profile_power_w * (1.0 + 0.015 * airspeed_m_s**2)
        parasitic = (
            0.5
            * self.air_density_kg_m3
            * self.parasitic_drag_coefficient
            * self.reference_area_m2
            * airspeed_m_s**3
        )
        potential = self.mass_kg * payload_multiplier * self.gravity_m_s2 * climb_rate_m_s
        climb = potential / self.drivetrain_efficiency if potential >= 0 else potential * self.descent_recovery_factor
        nominal = max(0.0, induced + profile + parasitic + climb + self.avionics_power_w)
        total = nominal * safety_factor
        return PowerBreakdown(
            induced_w=induced,
            profile_w=profile,
            parasitic_w=parasitic,
            climb_w=climb,
            avionics_w=self.avionics_power_w,
            total_w=total,
            standard_deviation_w=total * self.relative_uncertainty,
        )
