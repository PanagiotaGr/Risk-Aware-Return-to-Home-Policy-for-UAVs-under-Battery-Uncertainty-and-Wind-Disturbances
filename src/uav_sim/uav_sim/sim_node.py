import rclpy
from rclpy.node import Node
import numpy as np

import csv
import os
from datetime import datetime

from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import UInt8

from uav_interfaces.msg import UavState  # ✅ correct import

DT = 0.1


class UAVSim(Node):
    def __init__(self):
        super().__init__('uav_sim')

        # -------------------------
        # ROS parameters (editable at runtime via --ros-args -p ...)
        # -------------------------
        self.declare_parameter('gust_prob', 0.05)          # probability per tick
        self.declare_parameter('wind_sigma', 0.5)          # gust magnitude (std)
        self.declare_parameter('meas_sigma', 0.02)         # battery measurement noise std

        self.declare_parameter('alpha', 0.01)              # drain per commanded speed
        self.declare_parameter('beta', 0.02)               # drain per wind magnitude
        self.declare_parameter('gamma', 0.001)             # baseline drain

        self.declare_parameter('log_dir', os.path.expanduser('~/uav_ws/logs'))

        # read params
        self.gust_prob = float(self.get_parameter('gust_prob').value)
        self.wind_sigma = float(self.get_parameter('wind_sigma').value)
        self.meas_sigma = float(self.get_parameter('meas_sigma').value)

        self.alpha = float(self.get_parameter('alpha').value)
        self.beta = float(self.get_parameter('beta').value)
        self.gamma = float(self.get_parameter('gamma').value)

        self.log_dir = str(self.get_parameter('log_dir').value)

        # -------------------------
        # State
        # -------------------------
        self.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        self.vel_cmd = Twist()

        self.battery_true = 1.0
        self.battery_hat = 1.0

        self.wind = np.array([0.0, 0.0], dtype=float)

        # mode: 0=MISSION, 1=RETURN_HOME (planner sets it)
        self.mode = 0

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.cmd_sub = self.create_subscription(
            Twist, '/uav/cmd_vel', self.cmd_cb, 10
        )
        self.mode_sub = self.create_subscription(
            UInt8, '/uav/mode', self.mode_cb, 10
        )

        self.state_pub = self.create_publisher(
            UavState, '/uav/state', 10
        )

        # -------------------------
        # Logging
        # -------------------------
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # (προαιρετικό) “tagging” στο filename με βασικά params
        tag = f"gp{self.gust_prob}_ws{self.wind_sigma}_ms{self.meas_sigma}"
        self.log_path = os.path.join(self.log_dir, f'run_{tag}_{stamp}.csv')

        self.log_f = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.log_f)
        self.writer.writerow([
            't',
            'x', 'y',
            'vx_cmd', 'vy_cmd',
            'wind_x', 'wind_y',
            'battery_hat', 'battery_true',
            'mode'
        ])

        self.t = 0.0
        self.get_logger().info(f'Logging to: {self.log_path}')

        # timer
        self.timer = self.create_timer(DT, self.step)

    def cmd_cb(self, msg: Twist):
        self.vel_cmd = msg

    def mode_cb(self, msg: UInt8):
        self.mode = int(msg.data)

    def step(self):
        # ---- wind + gusts ----
        if np.random.rand() < self.gust_prob:
            self.wind = np.random.normal(0.0, self.wind_sigma, size=2)

        # ---- kinematics ----
        v_cmd = np.array([self.vel_cmd.linear.x, self.vel_cmd.linear.y], dtype=float)
        p_dot = v_cmd + self.wind

        self.pose.x += float(p_dot[0] * DT)
        self.pose.y += float(p_dot[1] * DT)

        # ---- battery drain ----
        drain_rate = self.alpha * np.linalg.norm(v_cmd) + self.beta * np.linalg.norm(self.wind) + self.gamma
        self.battery_true = max(0.0, self.battery_true - float(drain_rate * DT))

        # ---- battery estimate (noisy) ----
        noise = float(np.random.normal(0.0, self.meas_sigma))
        self.battery_hat = float(np.clip(self.battery_true + noise, 0.0, 1.0))

        # ---- publish ----
        st = UavState()
        st.pose = self.pose
        st.velocity = self.vel_cmd
        st.battery_hat = float(self.battery_hat)
        st.battery_true = float(self.battery_true)
        st.mode = int(self.mode)

        self.state_pub.publish(st)

        # ---- log ----
        self.writer.writerow([
            self.t,
            self.pose.x, self.pose.y,
            self.vel_cmd.linear.x, self.vel_cmd.linear.y,
            float(self.wind[0]), float(self.wind[1]),
            float(self.battery_hat), float(self.battery_true),
            int(self.mode)
        ])
        self.t += DT

    def destroy_node(self):
        try:
            self.log_f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = UAVSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
