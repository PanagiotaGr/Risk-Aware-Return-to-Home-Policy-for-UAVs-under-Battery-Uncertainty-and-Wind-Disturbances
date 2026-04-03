"""
uav_rl/agent.py
----------------
Αντιστοιχεί στο ROS 2 node: uav_reinflearning/agent_node.py

Q-learning agent που μαθαίνει πότε να κάνει RTH.

State space (discretized):
    s = (battery_bin, dist_bin, wind_bin)

Action space:
    0 = CONTINUE MISSION
    1 = RETURN HOME

Reward:
    +10   αν φτάσει goal
    +5    αν επιστρέψει σπίτι με ασφάλεια
    -20   αν εξαντληθεί η μπαταρία
    -0.1  ανά tick (κόστος χρόνου)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Dict
import json
import os


@dataclass
class RLAgentParams:
    # State space bins
    battery_bins: int = 10          # 0.0–1.0  σε 10 bins
    dist_bins: int = 10             # 0–10m    σε 10 bins
    wind_bins: int = 5              # 0–3 m/s  σε 5 bins

    max_dist: float = 10.0
    max_wind: float = 3.0

    # Q-learning hyperparameters
    alpha: float = 0.1              # learning rate
    gamma: float = 0.95             # discount factor
    epsilon: float = 1.0            # initial exploration rate
    epsilon_min: float = 0.05
    epsilon_decay: float = 0.995

    # Rewards
    r_goal: float = 10.0
    r_safe_return: float = 5.0
    r_crash: float = -20.0
    r_step: float = -0.1

    n_actions: int = 2              # 0=MISSION, 1=RTH

    q_table_path: str = "./logs/q_table.json"


class QLearningAgent:
    """
    Tabular Q-learning agent για RTH decisions.

    Αντιστοιχία με ROS 2:
      control_loop()  ←→  timer callback (10 Hz)
      select_action() ←→  RL policy
      update()        ←→  Q-table update
    """

    def __init__(self, params: RLAgentParams):
        self.p = params

        # Q-table: shape = (battery_bins, dist_bins, wind_bins, n_actions)
        shape = (
            params.battery_bins,
            params.dist_bins,
            params.wind_bins,
            params.n_actions,
        )
        self.Q = np.zeros(shape, dtype=float)

        self.epsilon = params.epsilon
        self._episode_rewards = []
        self._current_episode_reward = 0.0

    # ------------------------------------------------------------------
    # State discretization
    # ------------------------------------------------------------------
    def _discretize(
        self,
        battery: float,
        dist: float,
        wind_mag: float,
    ) -> Tuple[int, int, int]:
        """Μετατρέπει continuous state → discrete bins."""
        b_bin = int(np.clip(battery * self.p.battery_bins, 0, self.p.battery_bins - 1))
        d_bin = int(np.clip(
            dist / self.p.max_dist * self.p.dist_bins,
            0, self.p.dist_bins - 1
        ))
        w_bin = int(np.clip(
            wind_mag / self.p.max_wind * self.p.wind_bins,
            0, self.p.wind_bins - 1
        ))
        return b_bin, d_bin, w_bin

    # ------------------------------------------------------------------
    # Policy  (ε-greedy)
    # ------------------------------------------------------------------
    def select_action(
        self,
        battery_hat: float,
        dist_home: float,
        wind_mag: float,
        deterministic: bool = False,
    ) -> int:
        """
        Επιλέγει action (0=MISSION, 1=RTH).

        deterministic=True → greedy policy (για evaluation)
        """
        state = self._discretize(battery_hat, dist_home, wind_mag)

        if not deterministic and np.random.rand() < self.epsilon:
            return int(np.random.randint(self.p.n_actions))  # explore
        return int(np.argmax(self.Q[state]))                 # exploit

    # ------------------------------------------------------------------
    # Q-learning update  (Bellman equation)
    # ------------------------------------------------------------------
    def update(
        self,
        battery_hat: float,
        dist_home: float,
        wind_mag: float,
        action: int,
        reward: float,
        battery_hat_next: float,
        dist_home_next: float,
        wind_mag_next: float,
        done: bool,
    ):
        """
        Q(s,a) ← Q(s,a) + α * [r + γ * max_a' Q(s',a') - Q(s,a)]
        """
        s  = self._discretize(battery_hat, dist_home, wind_mag)
        s_ = self._discretize(battery_hat_next, dist_home_next, wind_mag_next)

        q_current = self.Q[s][action]
        q_next    = 0.0 if done else float(np.max(self.Q[s_]))

        td_error = reward + self.p.gamma * q_next - q_current
        self.Q[s][action] += self.p.alpha * td_error

        self._current_episode_reward += reward

    def end_episode(self):
        """Καλείται στο τέλος κάθε episode."""
        self._episode_rewards.append(self._current_episode_reward)
        self._current_episode_reward = 0.0
        # decay epsilon
        self.epsilon = max(
            self.p.epsilon_min,
            self.epsilon * self.p.epsilon_decay
        )

    # ------------------------------------------------------------------
    # Save / Load
    # ------------------------------------------------------------------
    def save(self, path: str = None):
        path = path or self.p.q_table_path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        data = {
            "Q": self.Q.tolist(),
            "epsilon": self.epsilon,
            "episode_rewards": self._episode_rewards,
        }
        with open(path, "w") as f:
            json.dump(data, f)

    def load(self, path: str = None):
        path = path or self.p.q_table_path
        with open(path) as f:
            data = json.load(f)
        self.Q = np.array(data["Q"])
        self.epsilon = data["epsilon"]
        self._episode_rewards = data["episode_rewards"]

    @property
    def episode_rewards(self):
        return self._episode_rewards

    def q_table_stats(self) -> Dict:
        return {
            "shape": self.Q.shape,
            "min": float(self.Q.min()),
            "max": float(self.Q.max()),
            "nonzero": int(np.count_nonzero(self.Q)),
            "epsilon": round(self.epsilon, 4),
        }
