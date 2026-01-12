import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class UavRLAgent(Node):
    def __init__(self):
        super().__init__('uav_rl_agent')

        # Subscribers: state (π.χ. odometry + imu)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/uav/odom',      # προσαρμόζεις στο δικό σου topic
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/uav/imu',       # προσαρμόζεις στο δικό σου topic
            self.imu_callback,
            10
        )

        # Publisher: actions προς UAV (π.χ. velocity commands)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/uav/cmd_vel',   # προσαρμόζεις στο δικό σου topic
            10
        )

        # Εσωτερική κατάσταση (state)
        self.last_odom = None
        self.last_imu = None

        # Timer για control loop (π.χ. 10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('UAV RL Agent node started.')

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg

    def imu_callback(self, msg: Imu):
        self.last_imu = msg

    def control_loop(self):
        # Εδώ θα μπει ο RL agent (policy π.χ. policy(state) -> action)

        if self.last_odom is None or self.last_imu is None:
            # Δεν έχουμε ακόμα πλήρες state
            return

        # --- Placeholder: απλό παράδειγμα "action" ---
        # Εδώ τώρα αντί για fixed action, θα βάζεις το RL policy output
        cmd = Twist()
        cmd.linear.x = 0.1   # π.χ. προχώρα λίγο μπροστά
        cmd.angular.z = 0.0  # χωρίς στροφή

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = UavRLAgent()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
