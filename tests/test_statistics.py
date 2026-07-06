from risk_rth_platform.statistics import binomial_ci95


def test_binomial_ci95_empty_sample_is_zero():
    assert binomial_ci95(0.5, 0) == 0.0


def test_binomial_ci95_is_zero_for_certain_rate():
    assert binomial_ci95(1.0, 100) == 0.0


def test_binomial_ci95_positive_for_uncertain_rate():
    assert binomial_ci95(0.5, 100) > 0.0
