"""
recorder_test.py — TESTING version of recorder.py.

This is the same as recorder.py but with the EKF subscription/storage/plot
blocks already UNCOMMENTED. Use this together with ekf_test.py to verify the
publishing pipeline works end-to-end before students do anything.

If you see green + red + blue lines in trajectory_plot.png, the pipeline
works. The blue (EKF) line will overlap the red (odom) line exactly because
ekf_test.py just relays odom unchanged.

USAGE
-----
Terminal 1: launch Gazebo
Terminal 2: python3 recorder_test.py
Terminal 3: python3 ekf_test.py
Terminal 4: python3 circle_driver.py

Then Ctrl+C in Terminal 2 to save the plot.
"""

import os
import csv
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class RecorderTest(Node):
    def __init__(self):
        super().__init__('recorder_test')

        self.truth_data = []
        self.odom_data  = []
        self.ekf_data   = []     # <-- enabled

        self.t0 = time.time()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Odometry, '/sim_ground_truth_pose',
            self.truth_callback, sensor_qos)

        self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10)

        # <-- EKF subscription enabled
        self.create_subscription(
            Odometry, '/ekf_pose',
            self.ekf_callback, 10)

        self.get_logger().info(
            'Recorder TEST started. Subscribing to truth + odom + ekf.')
        self.get_logger().info(
            'Press Ctrl+C to stop and save the plot.')

    def truth_callback(self, msg):
        t = time.time() - self.t0
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.truth_data.append((t, x, y))

    def odom_callback(self, msg):
        t = time.time() - self.t0
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_data.append((t, x, y))

    # <-- EKF callback enabled
    def ekf_callback(self, msg):
        t = time.time() - self.t0
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.ekf_data.append((t, x, y))

    def save_outputs(self):
        out_dir = os.getcwd()
        csv_path  = os.path.join(out_dir, 'trajectory_data.csv')
        plot_path = os.path.join(out_dir, 'trajectory_plot.png')

        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['source', 't', 'x', 'y'])
            for (t, x, y) in self.truth_data:
                writer.writerow(['truth', f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])
            for (t, x, y) in self.odom_data:
                writer.writerow(['odom',  f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])
            for (t, x, y) in self.ekf_data:
                writer.writerow(['ekf',   f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])

        print(f'Saved CSV to:  {csv_path}')
        print(f'  truth points: {len(self.truth_data)}, '
              f'odom points: {len(self.odom_data)}, '
              f'ekf points: {len(self.ekf_data)}')

        fig, ax = plt.subplots(figsize=(9, 9))

        if self.truth_data:
            xs = [d[1] for d in self.truth_data]
            ys = [d[2] for d in self.truth_data]
            ax.plot(xs, ys, 'g-', linewidth=2.5,
                    label=f'Ground truth ({len(self.truth_data)} pts)')

        if self.odom_data:
            xs = [d[1] for d in self.odom_data]
            ys = [d[2] for d in self.odom_data]
            ax.plot(xs, ys, 'r--', linewidth=1.5,
                    label=f'Odometry ({len(self.odom_data)} pts)')

        if self.ekf_data:
            xs = [d[1] for d in self.ekf_data]
            ys = [d[2] for d in self.ekf_data]
            ax.plot(xs, ys, 'b-', linewidth=1.5,
                    label=f'EKF estimate ({len(self.ekf_data)} pts)')

        beacons_x = [3.0,  0.0, -3.0]
        beacons_y = [0.0,  3.0,  1.5]
        ax.plot(beacons_x, beacons_y, 'k*', markersize=15,
                label='Known beacons')

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Robot trajectory: ground truth vs. odometry vs. EKF')
        ax.legend(loc='best')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(plot_path, dpi=110)
        plt.close()
        print(f'Saved plot to: {plot_path}')


def main():
    rclpy.init()
    node = RecorderTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nCtrl+C received, saving outputs...')
    finally:
        node.save_outputs()
        node.destroy_node()
        rclpy.shutdown()


main()
