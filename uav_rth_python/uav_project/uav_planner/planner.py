"""
uav_planner/planner.py
-----------------------
Αντιστοιχεί στο ROS 2 node: uav_planner/planner_node.py

Risk-aware RTH decision policy:
  Trigger RTH αν:  battery_hat <= E_home + z_delta * sigma_b

  όπου:
    E_home      = K_ENERGY_PER_M * dist_to_home   (energy proxy)
    z_delta     = Φ^-1(delta)  (π.χ. delta=0.95 → z=1.645)
    sigma_b     = εκτιμώμενη std της battery uncertainty

  Αυτό εξασφαλίζει ότι με πιθανότητα >= delta η μπαταρία
  επαρκεί για ασφαλή επιστροφή.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple

from uav_sim.state import UavState


@dataclass
class PlannerParams:
    """
    ROS 2 parameters του planner node.
    Αντιστοιχούν στα declare_parameter() του planner_node.py.
    """
    home: np.ndarray = None        # home position [x, y]
    goal: np.ndarray = None        # mission goal  [x, y]
    v_cmd: float = 1.0             # ταχύτητα πλοήγησης (m/s)

    # Risk-aware threshold parameters
    sigma_b: float = 0.05          # battery uncertainty std (planner-side assumption)
    delta: float = 0.95            # confidence level
    z_delta: float = 1.645         # Φ^-1(0.95) — z-score

    # Energy proxy: E_home = K * dist_home
    k_energy_per_m: float = 0.05

    def __post_init__(self):
        if self.home is None:
            self.home = np.array([0.0, 0.0])
        if self.goal is None:
            self.goal = np.array([5.0, 0.0])


class RiskAwarePlanner:
    """
    Risk-aware RTH planner.

    Αντιστοιχία με ROS 2:
      decide()    ←→  state_cb (subscription callback)
      get_cmd()   ←→  /uav/cmd_vel publication
      get_mode()  ←→  /uav/mode   publication
    """

    def __init__(self, params: PlannerParams):
        self.p = params
        self.mode = 0           # 0=MISSION, 1=RTH
        self._threshold_history = []

    # ------------------------------------------------------------------
    # Core decision  (αντιστοιχεί στο state_cb του planner_node.py)
    # ------------------------------------------------------------------
    def decide(self, state: UavState) -> Tuple[float, float, int]:
        """
        Δέχεται UavState, επιστρέφει (vx, vy, mode).

        Decision rule (από planner_node.py):
            threshold = energy_home + z_delta * sigma_b
            mode = RTH  if  battery_hat <= threshold
        """
        pos = state.pos
        bhat = state.battery_hat

        # ── energy-to-home proxy (distance-based) ─────────────────────
        dist_home = float(np.linalg.norm(self.p.home - pos))
        energy_home = self.p.k_energy_per_m * dist_home

        # ── risk-aware threshold ───────────────────────────────────────
        # battery_hat <= E_home + z*σ  → RTH
        threshold = energy_home + self.p.z_delta * self.p.sigma_b
        self._threshold_history.append(threshold)

        # ── mode decision ─────────────────────────────────────────────
        if self.mode == 0:  # MISSION
            if bhat <= threshold:
                self.mode = 1  # trigger RTH
        # Note: μόλις τεθεί RTH δεν ξαναγίνεται MISSION (latch)

        # ── navigation command ────────────────────────────────────────
        target = self.p.home if self.mode == 1 else self.p.goal
        direction = target - pos
        d = float(np.linalg.norm(direction))

        if d > 0.05:
            direction = direction / d
            vx = float(self.p.v_cmd * direction[0])
            vy = float(self.p.v_cmd * direction[1])
        else:
            vx, vy = 0.0, 0.0

        return vx, vy, self.mode

    def reset(self):
        self.mode = 0
        self._threshold_history.clear()

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def rth_threshold_at(self, pos: np.ndarray) -> float:
        """Υπολογίζει το RTH threshold για δεδομένη θέση."""
        dist = float(np.linalg.norm(self.p.home - pos))
        return self.p.k_energy_per_m * dist + self.p.z_delta * self.p.sigma_b

    def prob_safe_return_gaussian(self, bhat: float, pos: np.ndarray) -> float:
        """
        Αναλυτική Gaussian εκτίμηση P(ασφαλής επιστροφή).

        P(b_true >= E_home)  ≈  Φ((bhat - E_home) / sigma_b)
        """
        from scipy.stats import norm
        dist = float(np.linalg.norm(self.p.home - pos))
        energy_home = self.p.k_energy_per_m * dist
        return float(norm.cdf((bhat - energy_home) / self.p.sigma_b))
