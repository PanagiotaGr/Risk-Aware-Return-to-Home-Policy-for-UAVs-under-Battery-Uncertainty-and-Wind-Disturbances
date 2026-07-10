from __future__ import annotations

import pytest

from risk_rth.energy import MultirotorPowerModel, PathEnergyModel


@pytest.fixture
def model() -> MultirotorPowerModel:
    return MultirotorPowerModel(mass_kg=1.8, rotor_disk_area_m2=0.24)


def test_climb_increases_power(model: MultirotorPowerModel) -> None:
    level = model.estimate(airspeed_m_s=8.0, climb_rate_m_s=0.0)
    climb = model.estimate(airspeed_m_s=8.0, climb_rate_m_s=2.0)
    assert climb.total_w > level.total_w


def test_payload_increases_power(model: MultirotorPowerModel) -> None:
    nominal = model.estimate(airspeed_m_s=8.0, payload_multiplier=1.0)
    payload = model.estimate(airspeed_m_s=8.0, payload_multiplier=1.25)
    assert payload.total_w > nominal.total_w


def test_headwind_increases_path_energy(model: MultirotorPowerModel) -> None:
    path = PathEnergyModel(model)
    waypoints = [(0.0, 0.0, 40.0), (1000.0, 0.0, 40.0)]
    calm = path.estimate(waypoints, commanded_ground_speed_m_s=10.0)
    headwind = path.estimate(
        waypoints,
        commanded_ground_speed_m_s=10.0,
        wind_xy_m_s=(-5.0, 0.0),
    )
    assert headwind.expected_energy_wh > calm.expected_energy_wh
    assert headwind.standard_deviation_wh > calm.standard_deviation_wh


def test_model_is_deterministic(model: MultirotorPowerModel) -> None:
    first = model.estimate(airspeed_m_s=7.5, climb_rate_m_s=0.5)
    second = model.estimate(airspeed_m_s=7.5, climb_rate_m_s=0.5)
    assert first == second
