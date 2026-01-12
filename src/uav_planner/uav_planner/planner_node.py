import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

from uav_interfaces.msg import UavState  # ✅ from uav_interfaces

HOME = np.array([0.0, 0.0], dtype=float)
GOAL = np.array([5.0, 0.0], dtype=float)

# Risk-aware parameters
SIGMA_B = 0.05     # assumed battery estimation sigma (planner-side)
DELTA = 0.95
Z_DELTA = 1.645    # approx Φ^-1(0.95)

# Energy proxy per meter (tune)
K_ENERGY_PER_M = 0.05

# Speed command
V_CMD = 1.0

class Planner(Node):
    def __init__(self):
        super().__init__('uav_planner')

        self.mode = 0  # 0=MISSION, 1=RETURN_HOME

        self.state_sub = self.create_subscription(
            UavState, '/uav/state', self.state_cb, 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, '/uav/cmd_vel', 10
        )
        self.mode_pub = self.create_publisher(
            UInt8, '/uav/mode', 10
        )

    def state_cb(self, msg: UavState):
        pos = np.array([msg.pose.x, msg.pose.y], dtype=float)
        bhat = float(msg.battery_hat)

        # --- compute "energy-to-home" proxy (distance-based) ---
        dist_home = float(np.linalg.norm(HOME - pos))
        energy_home = K_ENERGY_PER_M * dist_home

        # risk-aware threshold: bhat <= E_home + z*sigma
        threshold = energy_home + Z_DELTA * SIGMA_B

        # mode decision
        self.mode = 1 if bhat <= threshold else 0

        # publish mode to sim (for logging/consistency)
        m = UInt8()
        m.data = int(self.mode)
        self.mode_pub.publish(m)

        # choose target
        target = HOME if self.mode == 1 else GOAL
        direction = target - pos
        d = float(np.linalg.norm(direction))

        cmd = Twist()
        if d > 0.1:
            direction = direction / d
            cmd.linear.x = float(V_CMD * direction[0])
            cmd.linear.y = float(V_CMD * direction[1])
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = Planner()
    rclpy.spin(node)
    rclpy.shutdown()
