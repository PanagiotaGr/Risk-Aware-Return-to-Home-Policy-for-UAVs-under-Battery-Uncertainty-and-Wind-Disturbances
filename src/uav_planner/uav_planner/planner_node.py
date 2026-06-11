#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from uav_interfaces.msg import UavState, BatteryState, WindState, RTHDecision


class RiskAwarePlanner(Node):
    def __init__(self):
        super().__init__('uav_planner')

        self.risk_threshold = 0.40
        self.num_samples = 500

        self.state = None
        self.battery = None
        self.wind = None

        self.create_subscription(UavState, '/uav/state', self.state_callback, 10)
        self.create_subscription(BatteryState, '/uav/battery', self.battery_callback, 10)
        self.create_subscription(WindState, '/uav/wind', self.wind_callback, 10)

        self.decision_pub = self.create_publisher(RTHDecision, '/uav/rth_decision', 10)

        self.timer = self.create_timer(0.2, self.evaluate_risk)

        self.get_logger().info('Risk-aware RTH planner started.')

    def state_callback(self, msg):
        self.state = msg

    def battery_callback(self, msg):
        self.battery = msg

    def wind_callback(self, msg):
        self.wind = msg

    def estimate_return_energy(self, distance, wind_speed, gust_factor):
        base_energy_per_meter = 0.08
        wind_penalty = 1.0 + 0.08 * max(0.0, wind_speed - 2.0)
        gust_penalty = 1.0 + 0.15 * max(0.0, gust_factor - 1.0)
        return distance * base_energy_per_meter * wind_penalty * gust_penalty

    def monte_carlo_safe_return_probability(self):
        distance = self.state.distance_to_home

        soc_mean = self.battery.soc
        soc_std = max(0.001, self.battery.soc_uncertainty)

        wind_mean = self.wind.speed
        gust_mean = self.wind.gust_factor

        feasible = 0

        for _ in range(self.num_samples):
            sampled_soc = min(1.0, max(0.0, random.gauss(soc_mean, soc_std)))
            sampled_wind = max(0.0, random.gauss(wind_mean, 0.8))
            sampled_gust = max(1.0, random.gauss(gust_mean, 0.15))

            available_energy = sampled_soc * 100.0
            required_energy = self.estimate_return_energy(
                distance,
                sampled_wind,
                sampled_gust
            )

            safety_margin = 8.0

            if available_energy > required_energy + safety_margin:
                feasible += 1

        return feasible / self.num_samples

    def evaluate_risk(self):
        if self.state is None or self.battery is None or self.wind is None:
            return

        safe_prob = self.monte_carlo_safe_return_probability()
        trigger = safe_prob < self.risk_threshold

        msg = RTHDecision()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.trigger_rth = trigger
        msg.safe_return_probability = safe_prob
        msg.risk_threshold = self.risk_threshold

        if trigger:
            msg.reason = 'Safe return probability below risk threshold'
        else:
            msg.reason = 'Mission continuation acceptable'

        self.decision_pub.publish(msg)

        self.get_logger().info(
            f'P_safe={safe_prob:.3f}, threshold={self.risk_threshold:.2f}, RTH={trigger}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RiskAwarePlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
