#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from uav_interfaces.msg import UavState, BatteryState, WindState


class UAVSimNode(Node):
    def __init__(self):
        super().__init__('uav_sim')

        self.state_pub = self.create_publisher(UavState, '/uav/state', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/uav/battery', 10)
        self.wind_pub = self.create_publisher(WindState, '/uav/wind', 10)

        self.timer = self.create_timer(0.1, self.update)

        self.x = 0.0
        self.y = 0.0
        self.z = 10.0

        self.vx = 1.5
        self.vy = 0.0
        self.vz = 0.0

        self.heading = 0.0
        self.soc = 1.0
        self.mode = 'MISSION'

        self.home_x = 0.0
        self.home_y = 0.0

        self.get_logger().info('UAV simulator node started.')

    def update(self):
        dt = 0.1

        wind_speed = 3.0 + random.gauss(0.0, 0.4)
        wind_direction = 180.0 + random.gauss(0.0, 8.0)
        gust_factor = max(1.0, random.gauss(1.15, 0.12))

        wind_rad = math.radians(wind_direction)
        wind_x = wind_speed * math.cos(wind_rad) * 0.05
        wind_y = wind_speed * math.sin(wind_rad) * 0.05

        if self.mode == 'MISSION':
            self.x += (self.vx + wind_x) * dt
            self.y += (self.vy + wind_y) * dt
        else:
            dx = self.home_x - self.x
            dy = self.home_y - self.y
            dist = math.hypot(dx, dy)

            if dist > 0.1:
                self.x += 2.0 * dx / dist * dt
                self.y += 2.0 * dy / dist * dt

        distance_to_home = math.hypot(self.x - self.home_x, self.y - self.home_y)

        base_drain = 0.00045
        wind_penalty = 0.00008 * max(0.0, wind_speed - 2.0)
        gust_penalty = 0.00005 * (gust_factor - 1.0)
        self.soc = max(0.0, self.soc - base_drain - wind_penalty - gust_penalty)

        stamp = self.get_clock().now().to_msg()

        state_msg = UavState()
        state_msg.header.stamp = stamp
        state_msg.header.frame_id = 'map'
        state_msg.x = self.x
        state_msg.y = self.y
        state_msg.z = self.z
        state_msg.vx = self.vx
        state_msg.vy = self.vy
        state_msg.vz = self.vz
        state_msg.heading = self.heading
        state_msg.distance_to_home = distance_to_home
        state_msg.mission_mode = self.mode

        battery_msg = BatteryState()
        battery_msg.header.stamp = stamp
        battery_msg.header.frame_id = 'base_link'
        battery_msg.soc = self.soc
        battery_msg.soc_uncertainty = 0.04
        battery_msg.estimated_remaining_energy = self.soc * 100.0

        wind_msg = WindState()
        wind_msg.header.stamp = stamp
        wind_msg.header.frame_id = 'map'
        wind_msg.speed = wind_speed
        wind_msg.direction = wind_direction
        wind_msg.gust_factor = gust_factor

        self.state_pub.publish(state_msg)
        self.battery_pub.publish(battery_msg)
        self.wind_pub.publish(wind_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UAVSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
