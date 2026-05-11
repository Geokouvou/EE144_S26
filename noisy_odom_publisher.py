"""
noisy_odom_publisher.py — adds Gaussian noise on top of the robot's /odom
topic and republishes the result on /noisy_odom.

Used in EE144 Lab 3 to demonstrate UKF's robustness vs. EKF when odometry
is much noisier than its declared noise model expects.

Tunables at the top of the file:
  EXTRA_SIGMA_V  -- extra std-dev of linear velocity noise (m/s)
  EXTRA_SIGMA_W  -- extra std-dev of angular velocity noise (rad/s)

USAGE
-----
Run alongside Gazebo. Then your UKF node should subscribe to /noisy_odom
instead of /odom (see the manual section 2.6).

    python3 noisy_odom_publisher.py
"""

import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry


# ============================================================================
#  Noise to ADD on top of whatever the real /odom topic already reports.
#  These are deliberately much larger than the EKF/UKF's modeled noise so
#  the difference between the filters becomes visible.
# ============================================================================
EXTRA_SIGMA_V = 0.20    # m/s  (very noisy linear velocity)
EXTRA_SIGMA_W = 0.20    # rad/s (very noisy angular velocity)


class NoisyOdomPublisher(Node):
    def __init__(self):
        super().__init__('noisy_odom_publisher')

        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.pub = self.create_publisher(
            Odometry, '/noisy_odom', 10)

        self.msg_count = 0
        self.get_logger().info(
            f'Noisy odom publisher started. '
            f'Adding sigma_v={EXTRA_SIGMA_V}, sigma_w={EXTRA_SIGMA_W}')

    def odom_callback(self, msg):
        # Copy the incoming message and add noise to the velocity fields.
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose       # leave pose untouched (filter uses twist)

        # Twist with added Gaussian noise
        out.twist.twist.linear.x  = (
            msg.twist.twist.linear.x  + float(np.random.randn() * EXTRA_SIGMA_V))
        out.twist.twist.linear.y  = msg.twist.twist.linear.y
        out.twist.twist.linear.z  = msg.twist.twist.linear.z
        out.twist.twist.angular.x = msg.twist.twist.angular.x
        out.twist.twist.angular.y = msg.twist.twist.angular.y
        out.twist.twist.angular.z = (
            msg.twist.twist.angular.z + float(np.random.randn() * EXTRA_SIGMA_W))
        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)

        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(
                f'  Published {self.msg_count} noisy odom messages')


def main():
    rclpy.init()
    node = NoisyOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
