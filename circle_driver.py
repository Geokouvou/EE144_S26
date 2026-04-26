"""
circle_driver.py

Drives the Turtlebot4 in a circle inside the beacons world, so the LIDAR
can see the three landmarks as the robot moves around them.

Circle parameters: radius ~2 m, period ~26 s per loop, 3 loops total (~78 s).

Beacons are at (3, 0), (0, 3), and (-3, 1.5). A circle of radius 2 m centered
at the origin keeps the robot close enough to all three beacons that the
LIDAR sees them throughout the motion.
"""

import rclpy
import threading
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.action import Undock


class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        self.vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.undock_action_client = ActionClient(self, Undock, '/undock')
        self.get_logger().info('Circle driver started!')

    def undock(self):
        self.undock_action_client.wait_for_server()
        undock_goal_result = self.undock_action_client.send_goal(Undock.Goal())
        if undock_goal_result.result.is_docked:
            self.get_logger().error('Undocking failed')

    def run(self):
        # Circle parameters
        # ----------------------------------------------------------------
        # linear  = forward speed (m/s)
        # angular = turn rate (rad/s)
        # radius  = linear / angular   --->  0.5 / 0.25 = 2.0 m
        # period  = 2*pi / angular     --->  ~25 s per full loop
        linear  = 0.5
        angular = 0.25
        rate_hz = 10
        loops   = 10
        period  = 2 * 3.14159 / angular        # seconds per loop
        n_steps = int(period * loops * rate_hz)

        # First undock so the robot is free to move
        self.undock()
        time.sleep(1)

        vel = TwistStamped()
        vel.twist.linear.x  = linear
        vel.twist.angular.z = angular

        self.get_logger().info(
            f'Driving circle: radius={linear/angular:.2f} m, '
            f'{loops} loops, ~{period * loops:.1f} s total')

        for _ in range(n_steps):
            vel.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(vel)
            time.sleep(1.0 / rate_hz)

        # Stop the robot at the end
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        self.vel_pub.publish(stop)
        self.get_logger().info('Circle complete. Stopping.')


def main(args=None):
    rclpy.init(args=args)
    node = CircleDriver()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    time.sleep(5)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
