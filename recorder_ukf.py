"""
recorder.py — records robot trajectories for EE144 Lab 3 (UKF).

Subscribes to:
  /sim_ground_truth_pose  (Gazebo ground truth)
  /odom                   (wheel odometry, drifts)
  /ukf_pose               (your UKF estimate -- see TODO blocks below)

OPTIONAL blocks (commented out, uncomment as the lab progresses):
  /noisy_odom             (section 2.6 -- the noisy-odom experiment)
  /ekf_pose               (BONUS -- run your Lab 2 EKF alongside UKF)

On Ctrl+C, saves a CSV and a plot of all subscribed sources.

USAGE
-----
Terminal 1: launch Gazebo
Terminal 2: python3 recorder.py
Terminal 3: python3 ukf.py
Terminal 4: python3 circle_driver.py
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


class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')

        # Storage for samples. Each entry is (t, x, y).
        self.truth_data = []
        self.odom_data  = []
        self.ukf_data   = []

        # ====================================================================
        # TODO (Section 2.6 -- noisy odom experiment): uncomment to also store
        # noisy_odom samples.
        # ====================================================================
        # self.noisy_odom_data = []

        # ====================================================================
        # TODO (BONUS -- +5 points): uncomment to also store EKF samples.
        # Requires you to have ekf.py from Lab 2 running.
        # ====================================================================
        # self.ekf_data = []

        self.t0 = time.time()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Ground truth subscription ----
        self.create_subscription(
            Odometry, '/sim_ground_truth_pose',
            self.truth_callback, sensor_qos)

        # ---- Odometry subscription ----
        self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10)

        # ---- UKF subscription ----
        self.create_subscription(
            Odometry, '/ukf_pose',
            self.ukf_callback, 10)

        # ====================================================================
        # TODO (Section 2.6): uncomment to also subscribe to /noisy_odom.
        # ====================================================================
        # self.create_subscription(
        #     Odometry, '/noisy_odom',
        #     self.noisy_odom_callback, 10)

        # ====================================================================
        # TODO (BONUS): uncomment to also subscribe to /ekf_pose.
        # ====================================================================
        # self.create_subscription(
        #     Odometry, '/ekf_pose',
        #     self.ekf_callback, 10)

        self.get_logger().info('Recorder started.')
        self.get_logger().info('Press Ctrl+C to stop and save the plot.')

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

    def ukf_callback(self, msg):
        t = time.time() - self.t0
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.ukf_data.append((t, x, y))

    # ========================================================================
    # TODO (Section 2.6): uncomment the noisy_odom callback below.
    # ========================================================================
    # def noisy_odom_callback(self, msg):
    #     t = time.time() - self.t0
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     self.noisy_odom_data.append((t, x, y))

    # ========================================================================
    # TODO (BONUS): uncomment the ekf callback below.
    # ========================================================================
    # def ekf_callback(self, msg):
    #     t = time.time() - self.t0
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     self.ekf_data.append((t, x, y))

    def save_outputs(self):
        out_dir = os.getcwd()
        csv_path  = os.path.join(out_dir, 'trajectory_data.csv')
        plot_path = os.path.join(out_dir, 'trajectory_plot.png')

        # ---- Save CSV ----
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['source', 't', 'x', 'y'])
            for (t, x, y) in self.truth_data:
                writer.writerow(['truth', f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])
            for (t, x, y) in self.odom_data:
                writer.writerow(['odom',  f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])
            for (t, x, y) in self.ukf_data:
                writer.writerow(['ukf',   f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])

            # ================================================================
            # TODO (Section 2.6): uncomment to also save noisy_odom samples.
            # ================================================================
            # for (t, x, y) in self.noisy_odom_data:
            #     writer.writerow(['noisy_odom', f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])

            # ================================================================
            # TODO (BONUS): uncomment to also save EKF samples.
            # ================================================================
            # for (t, x, y) in self.ekf_data:
            #     writer.writerow(['ekf', f'{t:.3f}', f'{x:.4f}', f'{y:.4f}'])

        print(f'Saved CSV to:  {csv_path}')
        print(f'  truth: {len(self.truth_data)},'
              f' odom: {len(self.odom_data)},'
              f' ukf: {len(self.ukf_data)}')

        # ====================================================================
        # TODO (Section 2.6): uncomment to also print noisy_odom count.
        # ====================================================================
        # print(f'  noisy_odom: {len(self.noisy_odom_data)}')

        # ====================================================================
        # TODO (BONUS): uncomment to also print EKF count.
        # ====================================================================
        # print(f'  ekf: {len(self.ekf_data)}')

        # ---- Save plot ----
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

        if self.ukf_data:
            xs = [d[1] for d in self.ukf_data]
            ys = [d[2] for d in self.ukf_data]
            ax.plot(xs, ys, 'b-', linewidth=1.5,
                    label=f'UKF estimate ({len(self.ukf_data)} pts)')

        # ====================================================================
        # TODO (Section 2.6): uncomment to also plot the noisy_odom trail.
        # ====================================================================
        # if self.noisy_odom_data:
        #     xs = [d[1] for d in self.noisy_odom_data]
        #     ys = [d[2] for d in self.noisy_odom_data]
        #     ax.plot(xs, ys, color='orange', linestyle=':', linewidth=1.0,
        #             label=f'Noisy odom ({len(self.noisy_odom_data)} pts)')

        # ====================================================================
        # TODO (BONUS): uncomment to also plot the EKF trail.
        # ====================================================================
        # if self.ekf_data:
        #     xs = [d[1] for d in self.ekf_data]
        #     ys = [d[2] for d in self.ekf_data]
        #     ax.plot(xs, ys, color='purple', linewidth=1.5,
        #             label=f'EKF estimate ({len(self.ekf_data)} pts)')

        # Mark known beacon positions
        beacons_x = [3.0,  0.0, -3.0]
        beacons_y = [0.0,  3.0,  1.5]
        ax.plot(beacons_x, beacons_y, 'k*', markersize=15,
                label='Known beacons')

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Robot trajectory: ground truth vs. odometry vs. UKF')
        ax.legend(loc='best')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(plot_path, dpi=110)
        plt.close()
        print(f'Saved plot to: {plot_path}')


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nCtrl+C received, saving outputs...')
    finally:
        node.save_outputs()
        node.destroy_node()
        rclpy.shutdown()


main()
