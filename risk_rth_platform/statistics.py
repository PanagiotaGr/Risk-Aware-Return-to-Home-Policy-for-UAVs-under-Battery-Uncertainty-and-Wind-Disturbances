from __future__ import annotations

import math


def binomial_ci95(rate: float, n: int) -> float:
    if n <= 0:
        return 0.0
    bounded = min(1.0, max(0.0, rate))
    return 1.96 * math.sqrt((bounded * (1.0 - bounded)) / n)
