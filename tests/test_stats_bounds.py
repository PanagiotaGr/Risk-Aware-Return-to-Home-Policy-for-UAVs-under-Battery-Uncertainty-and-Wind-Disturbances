from risk_rth_platform.statistics import binomial_ci95


def test_interval_is_bounded():
    value = binomial_ci95(0.5, 100)
    assert value > 0.0
    assert value < 1.0
