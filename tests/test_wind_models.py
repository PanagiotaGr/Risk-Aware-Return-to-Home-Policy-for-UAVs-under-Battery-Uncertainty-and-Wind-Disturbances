from __future__ import annotations

import pytest

from risk_rth.wind import ConstantWindField, GaussianWindField, GridWindField, SinusoidalGustWindField


def test_constant_wind_field_returns_same_vector() -> None:
    field = ConstantWindField(u_mps=3.0, v_mps=-4.0)

    sample = field.sample(x_m=10.0, y_m=20.0, t_s=5.0)

    assert sample.vector.u == 3.0
    assert sample.vector.v == -4.0
    assert sample.speed_mps == pytest.approx(5.0)


def test_gaussian_wind_field_is_reproducible_at_same_coordinate() -> None:
    field = GaussianWindField(mean_u_mps=2.0, mean_v_mps=0.0, std_mps=0.5, seed=11)

    first = field.sample(x_m=1.234, y_m=5.678, t_s=9.101)
    second = field.sample(x_m=1.234, y_m=5.678, t_s=9.101)

    assert first.vector == second.vector


def test_sinusoidal_gust_field_validates_period() -> None:
    field = SinusoidalGustWindField(base_u_mps=1.0, base_v_mps=0.0, period_s=0.0)

    with pytest.raises(ValueError):
        field.sample(x_m=0.0, y_m=0.0, t_s=0.0)


def test_grid_wind_field_bilinear_interpolation() -> None:
    field = GridWindField(
        u_grid=((0.0, 2.0), (2.0, 4.0)),
        v_grid=((0.0, 0.0), (2.0, 2.0)),
        resolution_m=10.0,
    )

    sample = field.sample(x_m=5.0, y_m=5.0, t_s=0.0)

    assert sample.vector.u == pytest.approx(2.0)
    assert sample.vector.v == pytest.approx(1.0)


def test_grid_wind_field_clamps_outside_coordinates() -> None:
    field = GridWindField(
        u_grid=((1.0, 2.0), (3.0, 4.0)),
        v_grid=((0.0, 0.0), (0.0, 0.0)),
        resolution_m=10.0,
    )

    sample = field.sample(x_m=-100.0, y_m=-100.0, t_s=0.0)

    assert sample.vector.u == 1.0
