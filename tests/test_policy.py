import unittest

from risk_rth.models import UAVState, Uncertainty, estimate_safe_return_probability, should_trigger_rth


class RiskPolicyTests(unittest.TestCase):
    def test_high_battery_is_safer_than_low_battery(self):
        near_home = UAVState(x_m=500, y_m=0, soc_wh=80)
        low_battery = UAVState(x_m=500, y_m=0, soc_wh=10)
        unc = Uncertainty(wind_mean_mps=2.0, wind_sigma_mps=0.5, soc_sigma_wh=1.0)
        high = estimate_safe_return_probability(near_home, unc, samples=300, seed=1)
        low = estimate_safe_return_probability(low_battery, unc, samples=300, seed=1)
        self.assertGreater(high, low)

    def test_threshold_decision(self):
        self.assertTrue(should_trigger_rth(0.4, 0.7))
        self.assertFalse(should_trigger_rth(0.9, 0.7))


if __name__ == "__main__":
    unittest.main()
