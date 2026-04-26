"""
ekf_test.py — a SANITY-CHECK version of ekf.py.

This file is NOT the real EKF. It's a stub that lets you verify the
publishing pipeline (recorder + EKF node + circle driver) works end-to-end
BEFORE you start filling in the actual EKF math.

What it does:
- Subscribes to /odom (just like the real ekf.py)
- IGNORES the math entirely
- Just copies the incoming odometry pose into self.mu
- Publishes that pose on /ekf_pose

If you see a blue line in the recorder plot that EXACTLY overlaps the red
odometry line, the publishing pipeline is working. Then you can swap in
the real ekf.py with the math TODOs.

USAGE
-----
Same as the real ekf.py:
    python3 ekf_test.py
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


def yaw_from_quaternion(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class EKFNodeTest(Node):
    def __init__(self):
        super().__init__('ekf_node_test')

        # State (just for matching the structure of the real EKF)
        self.mu = np.array([0.0, 0.0, 0.0])

        # Subscribe to odom and publish to /ekf_pose
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pose_pub = self.create_publisher(Odometry, '/ekf_pose', 10)

        self.msg_count = 0
        self.get_logger().info('EKF TEST node started.')
        self.get_logger().info('  Will copy /odom -> /ekf_pose with no math.')
        self.get_logger().info('  If you see a blue line on top of the red')
        self.get_logger().info('  odom line in the recorder plot, plumbing works!')

    def odom_callback(self, msg):
        # Just copy the incoming pose into self.mu
        self.mu[0] = msg.pose.pose.position.x
        self.mu[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.mu[2] = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        # Publish it on /ekf_pose (this is the part we are testing)
        self.publish_pose(msg.header.stamp)

        self.msg_count += 1
        if self.msg_count % 50 == 0:
            self.get_logger().info(
                f'  Published {self.msg_count} pose messages on /ekf_pose')

    def publish_pose(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        mu = self.mu
        msg.pose.pose.position.x = float(mu[0])
        msg.pose.pose.position.y = float(mu[1])
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(float(mu[2]))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = EKFNodeTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
