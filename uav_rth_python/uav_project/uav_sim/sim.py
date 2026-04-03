"""
uav_sim/sim.py
--------------
Αντιστοιχεί στο ROS 2 node: uav_sim/sim_node.py

Φυσικό μοντέλο UAV:
  - κινηματική: pos += (v_cmd + wind) * dt
  - μπαταρία:   drain = α*|v| + β*|wind| + γ   (ανά dt)
  - measurement noise:  battery_hat = battery_true + N(0, meas_sigma)
  - gusts:       τυχαία gust με πιθανότητα gust_prob ανά tick
  - logging:     αποθηκεύει κάθε tick σε list (→ CSV/DataFrame)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import csv
import os

from .state import UavState


@dataclass
class SimParams:
    """
    ROS 2 παράμετροι που δηλώνονται με declare_parameter().
    """
    # wind
    gust_prob: float = 0.05     # πιθανότητα gust ανά tick
    wind_sigma: float = 0.5     # std της gust (m/s)
    wind_base: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))

    # battery measurement noise
    meas_sigma: float = 0.02

    # drain model:  drain = alpha*|v| + beta*|wind| + gamma
    alpha: float = 0.01         # κόστος ανά ταχύτητα
    beta: float  = 0.02         # κόστος ανά wind magnitude
    gamma: float = 0.001        # baseline drain

    # simulation
    dt: float = 0.1             # timestep (s)

    # αρχικές θέσεις
    start: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    home: np.ndarray  = field(default_factory=lambda: np.array([0.0, 0.0]))

    # log dir
    log_dir: str = "./logs"


class UAVSim:
    """
    Κλάση που υλοποιεί τον simulator loop.

    Αντιστοιχία με ROS 2:
      step()      ←→  timer callback (self.create_timer(DT, self.step))
      set_cmd()   ←→  /uav/cmd_vel  subscription
      set_mode()  ←→  /uav/mode     subscription
      get_state() ←→  /uav/state    publication
    """

    def __init__(self, params: SimParams, run_tag: str = "run"):
        self.p = params
        self.run_tag = run_tag

        # --- state ---
        self.pos = params.start.copy().astype(float)
        self.vel_cmd = np.zeros(2, dtype=float)
        self.wind = params.wind_base.copy().astype(float)
        self.battery_true = 1.0
        self.battery_hat = 1.0
        self.mode = 0
        self.t = 0.0

        # --- log ---
        self._log: List[dict] = []

        os.makedirs(params.log_dir, exist_ok=True)
        self._csv_path = os.path.join(params.log_dir, f"{run_tag}.csv")

    # ------------------------------------------------------------------
    # Setters  (αντικαθιστούν τα ROS 2 subscriptions)
    # ------------------------------------------------------------------
    def set_cmd(self, vx: float, vy: float):
        """Δέχεται velocity command από τον planner."""
        self.vel_cmd = np.array([vx, vy], dtype=float)

    def set_mode(self, mode: int):
        """0=MISSION, 1=RTH — ορίζεται από τον planner."""
        self.mode = mode

    # ------------------------------------------------------------------
    # Core step  (ένα timestep dt)
    # ------------------------------------------------------------------
    def step(self) -> UavState:
        """
        Εκτελεί ένα βήμα φυσικής και επιστρέφει UavState.

        Αντιστοιχεί στο ROS 2:
            def step(self):  # timer callback
        """
        # ── wind / gusts ──────────────────────────────────────────────
        if np.random.rand() < self.p.gust_prob:
            self.wind = np.random.normal(0.0, self.p.wind_sigma, size=2)
        else:
            # gentle drift back to base wind
            self.wind = self.p.wind_base + np.random.normal(0.0, self.p.wind_sigma * 0.1, size=2)

        # ── kinematics ───────────────────────────────────────────────
        effective_vel = self.vel_cmd + self.wind
        self.pos = self.pos + effective_vel * self.p.dt

        # ── battery drain ────────────────────────────────────────────
        drain = (
            self.p.alpha * np.linalg.norm(self.vel_cmd)
            + self.p.beta  * np.linalg.norm(self.wind)
            + self.p.gamma
        ) * self.p.dt
        self.battery_true = float(max(0.0, self.battery_true - drain))

        # ── noisy measurement ─────────────────────────────────────────
        noise = float(np.random.normal(0.0, self.p.meas_sigma))
        self.battery_hat = float(np.clip(self.battery_true + noise, 0.0, 1.0))

        self.t += self.p.dt

        # ── log ───────────────────────────────────────────────────────
        row = {
            "t": round(self.t, 4),
            "x": round(self.pos[0], 4),
            "y": round(self.pos[1], 4),
            "vx_cmd": round(self.vel_cmd[0], 4),
            "vy_cmd": round(self.vel_cmd[1], 4),
            "wind_x": round(self.wind[0], 4),
            "wind_y": round(self.wind[1], 4),
            "battery_hat": round(self.battery_hat, 6),
            "battery_true": round(self.battery_true, 6),
            "mode": self.mode,
        }
        self._log.append(row)

        return self._make_state()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _make_state(self) -> UavState:
        return UavState(
            x=float(self.pos[0]),
            y=float(self.pos[1]),
            vx=float(self.vel_cmd[0]),
            vy=float(self.vel_cmd[1]),
            battery_hat=self.battery_hat,
            battery_true=self.battery_true,
            mode=self.mode,
            t=self.t,
        )

    def get_state(self) -> UavState:
        return self._make_state()

    @property
    def log(self) -> List[dict]:
        return self._log

    def save_csv(self):
        """Αποθηκεύει το log σε CSV (αντίστοιχο με το logging του sim_node.py)."""
        if not self._log:
            return
        with open(self._csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self._log[0].keys())
            writer.writeheader()
            writer.writerows(self._log)

    @property
    def csv_path(self) -> str:
        return self._csv_path
