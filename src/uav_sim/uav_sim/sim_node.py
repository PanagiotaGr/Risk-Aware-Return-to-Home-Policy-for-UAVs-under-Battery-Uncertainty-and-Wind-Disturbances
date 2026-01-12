import rclpy
from rclpy.node import Node
import numpy as np

import csv
import os
from datetime import datetime

from geometry_msgs.msg import Twist, Pose2D
from uav_interfaces.msg import UavState  # ✅ from uav_interfaces
from std_msgs.msg import UInt8

DT = 0.1

class UAVSim(Node):
    def __init__(self):
        super().__init__('uav_sim')

        # --- state ---
        self.pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        self.vel = Twist()

        self.battery_true = 1.0
        self.battery_hat = 1.0

        # wind vector (m/s)
        self.wind = np.array([0.0, 0.0], dtype=float)

        # mode: 0=MISSION, 1=RETURN_HOME
        self.mode = 0

        # --- params (simple + stable) ---
        self.gust_prob = 0.05          # probability per tick
        self.wind_sigma = 0.5          # gust magnitude
        self.meas_sigma = 0.02         # battery measurement noise

        self.alpha = 0.01              # drain per speed
        self.beta = 0.02               # drain per wind magnitude
        self.gamma = 0.001             # baseline drain

        # --- ROS interfaces ---
        self.cmd_sub = self.create_subscription(
            Twist, '/uav/cmd_vel', self.cmd_cb, 10
        )
        self.mode_sub = self.create_subscription(
            UInt8, '/uav/mode', self.mode_cb, 10
        )

        self.state_pub = self.create_publisher(
            UavState, '/uav/state', 10
        )

        # --- logging ---
        log_dir = os.path.expanduser('~/uav_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(log_dir, f'run_{stamp}.csv')

        self.log_f = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.log_f)
        self.writer.writerow([
            't', 'x', 'y', 'vx_cmd', 'vy_cmd', 'wind_x', 'wind_y',
            'battery_hat', 'battery_true', 'mode'
        ])

        self.t = 0.0
        self.get_logger().info(f'Logging to {self.log_path}')

        self.timer = self.create_timer(DT, self.step)

    def cmd_cb(self, msg: Twist):
        self.vel = msg

    def mode_cb(self, msg: UInt8):
        self.mode = int(msg.data)

    def step(self):
        # --- wind gusts ---
        if np.random.rand() < self.gust_prob:
            self.wind = np.random.normal(0.0, self.wind_sigma, size=2)

        # --- kinematic update ---
        v_cmd = np.array([self.vel.linear.x, self.vel.linear.y], dtype=float)
        p_dot = v_cmd + self.wind

        self.pose.x += float(p_dot[0] * DT)
        self.pose.y += float(p_dot[1] * DT)

        # --- battery drain model ---
        drain_rate = self.alpha * np.linalg.norm(v_cmd) + self.beta * np.linalg.norm(self.wind) + self.gamma
        self.battery_true = max(0.0, self.battery_true - float(drain_rate * DT))

        # --- noisy estimate ---
        noise = float(np.random.normal(0.0, self.meas_sigma))
        self.battery_hat = float(np.clip(self.battery_true + noise, 0.0, 1.0))

        # --- publish state ---
        msg = UavState()
        msg.pose = self.pose
        msg.velocity = self.vel
        msg.battery_hat = float(self.battery_hat)
        msg.battery_true = float(self.battery_true)
        msg.mode = int(self.mode)

        self.state_pub.publish(msg)

        # --- log ---
        self.writer.writerow([
            self.t,
            self.pose.x, self.pose.y,
            self.vel.linear.x, self.vel.linear.y,
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
