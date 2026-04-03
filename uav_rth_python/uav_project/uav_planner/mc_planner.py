"""
uav_planner/mc_planner.py
--------------------------
Monte Carlo extension του risk-aware planner.

Αντί για την αναλυτική Gaussian προσέγγιση (z_delta * sigma_b),
χρησιμοποιεί Monte Carlo sampling για να εκτιμήσει:

    P(safe return) = P(battery_true - cost_to_home > margin)

λαμβάνοντας υπόψη:
  - battery uncertainty: b_true ~ N(battery_hat, sigma_b)
  - wind uncertainty:    wind_head ~ N(wind_est, sigma_w)
  - energy cost:         cost = K * dist + wind_factor * wind_head
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple
import time

from uav_sim.state import UavState


@dataclass
class MCPlannerParams:
    home: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    goal: np.ndarray = field(default_factory=lambda: np.array([5.0, 0.0]))
    v_cmd: float = 1.0

    # Risk threshold
    tau: float = 0.80               # RTH αν P(safe) < tau

    # MC sampling
    n_samples: int = 500            # Monte Carlo samples

    # Battery uncertainty
    sigma_b: float = 0.05

    # Wind uncertainty
    sigma_w: float = 0.5            # std εκτίμησης ανέμου
    wind_factor: float = 0.005      # επιπλέον κόστος ανά (m/s headwind) ανά μέτρο

    # Energy model
    k_energy_per_m: float = 0.05    # βασικό κόστος ανά μέτρο

    # Safety margin
    margin: float = 0.05            # ελάχιστο SoC για ασφαλή προσγείωση


class MCRiskAwarePlanner:
    """
    Monte Carlo risk-aware RTH planner.

    Σε κάθε tick:
      1. Κάνει N MC samples από τις κατανομές μπαταρίας και ανέμου
      2. Υπολογίζει P(safe return)
      3. Αν P < tau → RTH
    """

    def __init__(self, params: MCPlannerParams):
        self.p = params
        self.mode = 0
        self.last_p_safe = 1.0
        self._p_safe_history = []
        self._mc_timing = []

    def _estimate_p_safe(
        self,
        battery_hat: float,
        pos: np.ndarray,
        wind_est: np.ndarray,
    ) -> float:
        """
        Monte Carlo εκτίμηση P(safe return).

        Για κάθε sample:
          b_s   = battery_hat + N(0, sigma_b)      [battery sample]
          w_s   = |wind_est| + N(0, sigma_w)       [wind magnitude sample]
          cost  = K*dist + wind_factor * max(0, w_s * cos(θ))  * dist
          safe  = b_s - cost > margin
        """
        dist = float(np.linalg.norm(self.p.home - pos))
        if dist < 0.1:
            return 1.0

        # γωνία προς home
        angle_home = np.arctan2(
            self.p.home[1] - pos[1],
            self.p.home[0] - pos[0]
        )
        wind_dir = float(np.arctan2(wind_est[1], wind_est[0]))
        wind_mag = float(np.linalg.norm(wind_est))

        t0 = time.perf_counter()

        # Vectorized MC
        rng = np.random.default_rng()
        b_samples = battery_hat + rng.normal(0.0, self.p.sigma_b, self.p.n_samples)
        w_samples = np.maximum(0.0,
            wind_mag + rng.normal(0.0, self.p.sigma_w, self.p.n_samples)
        )

        # headwind component (θετικό = αντίθετος άνεμος)
        headwind = w_samples * np.cos(wind_dir - angle_home)

        # energy cost per sample
        cost = (
            self.p.k_energy_per_m * dist
            + self.p.wind_factor * np.maximum(0.0, headwind) * dist
        )

        safe_mask = (b_samples - cost) > self.p.margin
        p_safe = float(np.mean(safe_mask))

        self._mc_timing.append(time.perf_counter() - t0)
        return p_safe

    def decide(
        self,
        state: UavState,
        wind_est: np.ndarray = None,
    ) -> Tuple[float, float, int, float]:
        """
        Επιστρέφει (vx, vy, mode, p_safe).

        Parameters
        ----------
        state    : UavState από simulator
        wind_est : εκτίμηση wind vector (αν None → [0,0])
        """
        if wind_est is None:
            wind_est = np.zeros(2)

        pos = state.pos
        p_safe = self._estimate_p_safe(state.battery_hat, pos, wind_est)
        self.last_p_safe = p_safe
        self._p_safe_history.append(p_safe)

        # ── RTH trigger ───────────────────────────────────────────────
        if self.mode == 0 and p_safe < self.p.tau:
            self.mode = 1

        # ── navigation ───────────────────────────────────────────────
        target = self.p.home if self.mode == 1 else self.p.goal
        direction = target - pos
        d = float(np.linalg.norm(direction))

        if d > 0.05:
            direction = direction / d
            vx = float(self.p.v_cmd * direction[0])
            vy = float(self.p.v_cmd * direction[1])
        else:
            vx, vy = 0.0, 0.0

        return vx, vy, self.mode, p_safe

    def reset(self):
        self.mode = 0
        self.last_p_safe = 1.0
        self._p_safe_history.clear()
        self._mc_timing.clear()

    @property
    def avg_mc_time_ms(self) -> float:
        if not self._mc_timing:
            return 0.0
        return float(np.mean(self._mc_timing) * 1000)
