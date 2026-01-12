import math
import statistics
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue


class LidarNoiseAdaptivePlanner(Node):
    def __init__(self):
        super().__init__('lidar_noise_adaptive_planner')

        # Ποιο node του Nav2 θα πειράζουμε
        # Συνήθως: /controller_server
        self.target_node_name = self.declare_parameter(
            'target_node_name', 'controller_server'
        ).get_parameter_value().string_value

        # Τι params θα αλλάζουμε (ονομα Nav2 παραμέτρων)
        # Αυτά είναι παραδείγματα, προσαρμόζεις στα δικά σου nav2_params.yaml
        self.param_low_noise = {
            'FollowPath.max_vel_x': 0.22,
            'FollowPath.acc_lim_x': 2.5,
        }
        self.param_high_noise = {
            'FollowPath.max_vel_x': 0.10,
            'FollowPath.acc_lim_x': 1.0,
        }

        # Thresholds για variance
        self.low_noise_threshold = self.declare_parameter(
            'low_noise_threshold', 0.01
        ).get_parameter_value().double_value
        self.high_noise_threshold = self.declare_parameter(
            'high_noise_threshold', 0.05
        ).get_parameter_value().double_value

        # Laser subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',        # αν είναι άλλο topic στο sim σου, άλλαξέ το
            self.scan_callback,
            10
        )

        # Client για set_parameters στο Nav2 controller_server
        service_name = f'/{self.target_node_name}/set_parameters'
        self.param_client = self.create_client(SetParameters, service_name)

        self.get_logger().info(
            f'Adaptive planner node started. Will reconfigure: {self.target_node_name}'
        )

        # Κατάσταση θορύβου: 'low', 'medium', 'high'
        self.current_noise_state: Optional[str] = None

    def scan_callback(self, msg: LaserScan):
        # Φιλτράρουμε inf / NaN
        ranges = [r for r in msg.ranges if math.isfinite(r)]
        if len(ranges) < 5:
            return

        # Υπολογισμός variance
        try:
            var = statistics.pvariance(ranges)
        except statistics.StatisticsError:
            return

        # Πολύ απλό noise metric: μόνο variance
        noise_level = var

        # Κατάσταση θορύβου
        if noise_level < self.low_noise_threshold:
            noise_state = 'low'
        elif noise_level > self.high_noise_threshold:
            noise_state = 'high'
        else:
            noise_state = 'medium'

        # Log ανά κάποια scans
        self.get_logger().debug(
            f'Noise metric: {noise_level:.4f}, state={noise_state}'
        )

        # Αν δεν αλλάζει state, δεν κάνουμε τίποτα
        if noise_state == self.current_noise_state:
            return

        # Αν αλλάζει, κάνουμε update Nav2 params
        self.current_noise_state = noise_state
        if noise_state == 'low':
            self.get_logger().info(
                f'Noise LOW ({noise_level:.4f}) → applying low-noise params.'
            )
            self.apply_params(self.param_low_noise)
        elif noise_state == 'high':
            self.get_logger().info(
                f'Noise HIGH ({noise_level:.4f}) → applying high-noise params.'
            )
            self.apply_params(self.param_high_noise)
        else:
            # medium: μπορείς να βάλεις κάτι ενδιάμεσο ή τίποτα
            self.get_logger().info(
                f'Noise MEDIUM ({noise_level:.4f}) → keeping current params.'
            )

    def apply_params(self, param_dict):
        # Περιμένουμε να υπάρχει το service
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'SetParameters service for {self.target_node_name} not available.'
            )
            return

        req = SetParameters.Request()
        for full_name, value in param_dict.items():
            # Τα plugins του DWB είναι κάτω από FollowPath.
            # Στο YAML αυτό αντιστοιχεί σε key τύπου:
            # controller_server:
            #   ros__parameters:
            #     FollowPath:
            #       max_vel_x: ...
            # Εδώ δίνουμε το "FollowPath.max_vel_x" κλπ.
            param_msg = Parameter()
            param_msg.name = full_name
            param_value = ParameterValue()
            param_value.type = ParameterValue.TYPE_DOUBLE
            param_value.double_value = float(value)
            param_msg.value = param_value
            req.parameters.append(param_msg)

        future = self.param_client.call_async(req)

        def callback(fut):
            if fut.result() is not None:
                self.get_logger().info(
                    f'Updated parameters on {self.target_node_name}: {param_dict}'
                )
            else:
                self.get_logger().warn('Failed to set parameters.')

        future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = LidarNoiseAdaptivePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
