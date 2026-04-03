"""
uav_sim/state.py
-----------------
Αντικαθιστά το ROS 2 custom message uav_interfaces/UavState.
Απλό dataclass που μεταφέρεται μεταξύ simulator, planner και RL agent.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class UavState:
    """
    Αντιστοιχεί στο:
        geometry_msgs/Pose2D  pose
        geometry_msgs/Twist   velocity
        float32               battery_hat   (εκτίμηση με θόρυβο)
        float32               battery_true  (πραγματική)
        uint8                 mode          (0=MISSION, 1=RTH)
    """
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    battery_hat: float = 1.0
    battery_true: float = 1.0
    mode: int = 0             # 0 = MISSION, 1 = RETURN_HOME
    t: float = 0.0

    @property
    def pos(self) -> np.ndarray:
        return np.array([self.x, self.y])

    @property
    def vel(self) -> np.ndarray:
        return np.array([self.vx, self.vy])

    def __repr__(self):
        mode_str = "MISSION" if self.mode == 0 else "RTH"
        return (
            f"UavState(t={self.t:.2f}s  pos=({self.x:.2f},{self.y:.2f})  "
            f"batt_hat={self.battery_hat:.3f}  batt_true={self.battery_true:.3f}  "
            f"mode={mode_str})"
        )
