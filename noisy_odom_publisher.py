"""
noisy_odom_publisher.py — produces a /noisy_odom topic with realistically
drifting pose (not just noisy velocities).

Maintains its own dead-reckoned pose by integrating noisy velocity readings.
The UKF/EKF will see both noisy velocities AND a drifting position.
"""

import math
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry


EXTRA_SIGMA_V = 0.20
EXTRA_SIGMA_W = 0.20


class NoisyOdomPublisher(Node):
    def __init__(self):
        super().__init__('noisy_odom_publisher')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/noisy_odom', 10)

        # Maintain our own integrated pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_t = None
        self.msg_count = 0

        self.get_logger().info(
            f'Noisy odom publisher started. '
            f'Adding sigma_v={EXTRA_SIGMA_V}, sigma_w={EXTRA_SIGMA_W}')

    def odom_callback(self, msg):
        # Get the time
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 1.0:
            return

        # Read true velocities, add noise
        v_noisy = msg.twist.twist.linear.x  + float(np.random.randn() * EXTRA_SIGMA_V)
        w_noisy = msg.twist.twist.angular.z + float(np.random.randn() * EXTRA_SIGMA_W)

        # Integrate noisy velocities into our own pose
        if abs(w_noisy) < 1e-6:
            self.x += v_noisy * dt * math.cos(self.theta)
            self.y += v_noisy * dt * math.sin(self.theta)
        else:
            r = v_noisy / w_noisy
            theta_new = self.theta + w_noisy * dt
            self.x += r * (-math.sin(self.theta) + math.sin(theta_new))
            self.y += r * ( math.cos(self.theta) - math.cos(theta_new))
            self.theta = theta_new
        # Wrap theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Build the output message
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        # Pose: our integrated noisy pose
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.position.z = 0.0
        half = 0.5 * self.theta
        out.pose.pose.orientation.z = math.sin(half)
        out.pose.pose.orientation.w = math.cos(half)
        out.pose.covariance = msg.pose.covariance

        # Twist: noisy velocities (so a filter subscribing to this will
        # also see noisy v and omega)
        out.twist.twist.linear.x  = v_noisy
        out.twist.twist.angular.z = w_noisy
        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)

        self.msg_count += 1
        if self.msg_count % 100 == 0:
            self.get_logger().info(
                f'  Published {self.msg_count} noisy odom messages. '
                f'Pose: ({self.x:.2f}, {self.y:.2f})')


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
