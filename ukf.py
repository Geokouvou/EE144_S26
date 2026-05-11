"""
ukf.py — UKF localization node for the Turtlebot4 in the beacons world.

Subscribes:
    /odom   (nav_msgs/Odometry) -- wheel odometry, drives PREDICT
    /scan   (sensor_msgs/LaserScan) -- LIDAR, drives UPDATE

Publishes:
    /ukf_pose (nav_msgs/Odometry) -- the UKF's pose estimate

WHAT YOU NEED TO DO
-------------------
There are two methods you must complete:

    UKFLocalization.predict(self, v, omega, dt)
    UKFLocalization.update(self, z, landmark_xy)

Each has TODOs and references to the equation numbers from the lab manual.
DO NOT modify the rest of this file -- the ROS plumbing, the LIDAR clustering,
the landmark map, sigma point generation, and the publisher are all already
set up for you.

USAGE
-----
Once you fill in the methods, run (in a separate terminal from Gazebo):

    python3 ukf.py
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ============================================================================
#                       KNOWN LANDMARK MAP
# ============================================================================
LANDMARKS = np.array([
    [ 3.0,  0.0],     # red beacon
    [ 0.0,  3.0],     # green beacon
    [-3.0,  1.5],     # blue beacon
])


# ============================================================================
#                       NOISE PARAMETERS
# Tune these if the filter behaves badly:
#   - Increase SIGMA_V / SIGMA_W if filter is too confident in odometry
#   - Increase SIGMA_RANGE / SIGMA_BEARING if it's too jumpy on measurements
# ============================================================================
SIGMA_V       = 0.05
SIGMA_W       = 0.05
SIGMA_RANGE   = 0.10
SIGMA_BEARING = 0.05

MAHALANOBIS_GATE = 9.21   # chi-squared 99% confidence, 2 DOF

# ----------------------------------------------------------------------------
# UKF tuning parameters -- the standard "scaled unscented transform" values
# ----------------------------------------------------------------------------
N_STATE = 3
ALPHA   = 1e-3
BETA    = 2.0
KAPPA   = 0.0
LAMBDA  = ALPHA**2 * (N_STATE + KAPPA) - N_STATE


# ============================================================================
#                       HELPERS
# ============================================================================
def wrap_to_pi(angle):
    """Wrap an angle to the range [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def yaw_from_quaternion(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


# ============================================================================
#                       MOTION AND MEASUREMENT MODELS
# Same as in the EKF lab, but here we pass sigma points through them DIRECTLY
# (no Jacobians needed). These are already implemented for you.
# ============================================================================
def motion_model(state, v, omega, dt):
    """Push a single state vector through the motion model."""
    x, y, theta = state[0], state[1], state[2]
    if abs(omega) < 1e-6:
        new_x = x + v * dt * math.cos(theta)
        new_y = y + v * dt * math.sin(theta)
        new_theta = theta
    else:
        r = v / omega
        theta_new = theta + omega * dt
        new_x = x + r * (-math.sin(theta) + math.sin(theta_new))
        new_y = y + r * ( math.cos(theta) - math.cos(theta_new))
        new_theta = theta_new
    return np.array([new_x, new_y, wrap_to_pi(new_theta)])


def measurement_model(state, landmark_xy):
    """Push a single state through the measurement model."""
    x, y, theta = state[0], state[1], state[2]
    lx, ly = landmark_xy
    dx = lx - x
    dy = ly - y
    return np.array([math.hypot(dx, dy),
                     wrap_to_pi(math.atan2(dy, dx) - theta)])


# ============================================================================
#                       THE UKF CLASS
# This is where YOU write the math. Two methods to complete: predict, update.
# ============================================================================
class UKFLocalization:
    def __init__(self):
        self.mu = np.array([0.0, 0.0, 0.0])
        self.Sigma = np.diag([0.01, 0.01, 0.01])
        self.Q = np.diag([SIGMA_V ** 2 * 0.1,
                          SIGMA_V ** 2 * 0.1,
                          SIGMA_W ** 2 * 0.1])
        self.R = np.diag([SIGMA_RANGE ** 2, SIGMA_BEARING ** 2])
        self.weights_m, self.weights_c = self._compute_weights()

    # ------------------------------------------------------------------ #
    #  WEIGHTS for combining sigma points -- already implemented          #
    # ------------------------------------------------------------------ #
    def _compute_weights(self):
        n = N_STATE
        Wm = np.zeros(2 * n + 1)
        Wc = np.zeros(2 * n + 1)
        Wm[0] = LAMBDA / (n + LAMBDA)
        Wc[0] = LAMBDA / (n + LAMBDA) + (1 - ALPHA**2 + BETA)
        for i in range(1, 2 * n + 1):
            Wm[i] = 1.0 / (2 * (n + LAMBDA))
            Wc[i] = 1.0 / (2 * (n + LAMBDA))
        return Wm, Wc

    # ------------------------------------------------------------------ #
    #  SIGMA POINT GENERATION -- already implemented                      #
    #  Creates 2n+1 sample points around (mu, Sigma) using a matrix       #
    #  square root via Cholesky decomposition.                            #
    # ------------------------------------------------------------------ #
    def _generate_sigma_points(self, mu, Sigma):
        n = N_STATE
        sigma_points = np.zeros((2 * n + 1, n))
        try:
            L = np.linalg.cholesky((n + LAMBDA) * Sigma)
        except np.linalg.LinAlgError:
            L = np.linalg.cholesky((n + LAMBDA) * (Sigma + 1e-9 * np.eye(n)))

        sigma_points[0] = mu
        for i in range(n):
            sigma_points[i + 1]     = mu + L[:, i]
            sigma_points[n + i + 1] = mu - L[:, i]

        for i in range(2 * n + 1):
            sigma_points[i, 2] = wrap_to_pi(sigma_points[i, 2])
        return sigma_points

    # ------------------------------------------------------------------ #
    #  PREDICT STEP                                                       #
    # ------------------------------------------------------------------ #
    def predict(self, v, omega, dt):
        """
        Run the UKF predict step.
          v, omega : control inputs from odometry
          dt       : time elapsed since last predict
        Updates self.mu and self.Sigma in place.
        """
        n = N_STATE

        # 1. Generate sigma points from current belief (already done for you)
        sigma_points = self._generate_sigma_points(self.mu, self.Sigma)

        # ----------------------------------------------------------------
        # TODO 1: Push each sigma point through the motion model.
        # Hint: call motion_model(sigma_points[i], v, omega, dt) for each i,
        # store results in `propagated` (shape (2n+1, n)).
        # ----------------------------------------------------------------
        propagated = np.zeros_like(sigma_points)
        # YOUR CODE HERE  (Eq. 2 of the manual)

        # ----------------------------------------------------------------
        # TODO 2: Recover the predicted mean as the weighted sum of
        # propagated sigma points (Eq. 3a). Don't forget to wrap theta.
        # ----------------------------------------------------------------
        mu_pred = np.zeros(n)
        # YOUR CODE HERE
        mu_pred[2] = wrap_to_pi(mu_pred[2])

        # ----------------------------------------------------------------
        # TODO 3: Recover the predicted covariance (Eq. 3b).
        # Sum weights_c[i] * (propagated[i] - mu_pred)(propagated[i] - mu_pred)^T
        # then add the process noise Q.
        # IMPORTANT: wrap the theta component of each difference using
        # wrap_to_pi BEFORE taking the outer product.
        # ----------------------------------------------------------------
        Sigma_pred = np.zeros((n, n))
        # YOUR CODE HERE

        self.mu = mu_pred
        self.Sigma = Sigma_pred

    # ------------------------------------------------------------------ #
    #  UPDATE STEP                                                        #
    # ------------------------------------------------------------------ #
    def update(self, z, landmark_xy):
        """
        Run the UKF update step for one observed landmark.
          z           : np.array([range, bearing]) from LIDAR
          landmark_xy : (lx, ly) world position of matched landmark
        Updates self.mu and self.Sigma in place.
        """
        n = N_STATE

        # 1. Generate sigma points from predicted belief (already done)
        sigma_points = self._generate_sigma_points(self.mu, self.Sigma)

        # ----------------------------------------------------------------
        # TODO 4: Push each sigma point through the measurement model.
        # Each result is a 2-vector [range, bearing].
        # Store in Z_sigma (shape (2n+1, 2)).
        # ----------------------------------------------------------------
        Z_sigma = np.zeros((2 * n + 1, 2))
        # YOUR CODE HERE  (Eq. 4)

        # ----------------------------------------------------------------
        # TODO 5: Recover predicted measurement mean (Eq. 5a).
        # Weighted sum across sigma points. Wrap the bearing at the end.
        # ----------------------------------------------------------------
        z_hat = np.zeros(2)
        # YOUR CODE HERE
        z_hat[1] = wrap_to_pi(z_hat[1])

        # ----------------------------------------------------------------
        # TODO 6: Compute innovation covariance S (Eq. 5b).
        # S = sum(W_c[i] * (Z_sigma[i] - z_hat)(Z_sigma[i] - z_hat)^T) + R
        # Wrap the BEARING component of each difference before outer product.
        # ----------------------------------------------------------------
        S = np.zeros((2, 2))
        # YOUR CODE HERE

        # ----------------------------------------------------------------
        # TODO 7: Compute cross-covariance T between state and measurement
        # (Eq. 5c).
        # T = sum(W_c[i] * (sigma_points[i] - mu)(Z_sigma[i] - z_hat)^T)
        # Wrap the THETA component of state diff AND the BEARING component
        # of measurement diff before outer products.
        # ----------------------------------------------------------------
        T = np.zeros((n, 2))
        # YOUR CODE HERE

        # ----------------------------------------------------------------
        # TODO 8: Compute the Kalman gain K (Eq. 5d).
        # ----------------------------------------------------------------
        K = np.zeros((n, 2))
        # YOUR CODE HERE  (K = T @ inv(S))

        # ----------------------------------------------------------------
        # TODO 9: Update self.mu (Eq. 5e). Wrap bearing in innovation
        # BEFORE multiplying by K. Wrap mu[2] after.
        # ----------------------------------------------------------------
        innovation = z - z_hat
        innovation[1] = wrap_to_pi(innovation[1])
        # YOUR CODE HERE  (self.mu = ...)
        self.mu[2] = wrap_to_pi(self.mu[2])

        # ----------------------------------------------------------------
        # TODO 10: Update self.Sigma (Eq. 5f).
        # Sigma = Sigma - K @ S @ K^T
        # ----------------------------------------------------------------
        # YOUR CODE HERE

    # ------------------------------------------------------------------ #
    #  Mahalanobis distance for data association (already implemented)   #
    # ------------------------------------------------------------------ #
    def mahalanobis_distance_sq(self, z, landmark_xy):
        z_hat = measurement_model(self.mu, landmark_xy)
        innov = z - z_hat
        innov[1] = wrap_to_pi(innov[1])
        S_approx = self.R + np.array([
            [self.Sigma[0,0] + self.Sigma[1,1], 0.0],
            [0.0, self.Sigma[2,2]],
        ])
        try:
            return float(innov @ np.linalg.inv(S_approx) @ innov)
        except np.linalg.LinAlgError:
            return float('inf')


# ============================================================================
#                       LIDAR CLUSTERING (already implemented)
# ============================================================================
def cluster_scan(scan, max_gap=0.15, min_points=3, max_width=0.5):
    ranges = np.array(scan.ranges)
    n = len(ranges)
    if n == 0:
        return []
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    clusters, current, prev_xy = [], [], None
    for i in range(n):
        if not valid[i]:
            if len(current) >= min_points:
                clusters.append(current)
            current = []
            prev_xy = None
            continue
        r = ranges[i]
        a = angles[i]
        x = r * math.cos(a)
        y = r * math.sin(a)
        if prev_xy is None:
            current = [(x, y)]
        else:
            if math.hypot(x - prev_xy[0], y - prev_xy[1]) < max_gap:
                current.append((x, y))
            else:
                if len(current) >= min_points:
                    clusters.append(current)
                current = [(x, y)]
        prev_xy = (x, y)
    if len(current) >= min_points:
        clusters.append(current)
    detections = []
    for c in clusters:
        xs = np.array([p[0] for p in c])
        ys = np.array([p[1] for p in c])
        cx, cy = xs.mean(), ys.mean()
        width = math.hypot(xs.max() - xs.min(), ys.max() - ys.min())
        if width > max_width:
            continue
        detections.append((math.hypot(cx, cy), math.atan2(cy, cx)))
    return detections


# ============================================================================
#                       THE ROS NODE (already implemented)
# ============================================================================
class UKFNode(Node):
    def __init__(self):
        super().__init__('ukf_node')
        self.ukf = UKFLocalization()
        self.last_odom_time = None
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # By default subscribe to /odom. For the noisy-odom experiment in
        # section 2.6 of the manual, change '/odom' to '/noisy_odom' below.
        self.create_subscription(Odometry, '/odom',
                                 self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self.scan_callback, sensor_qos)
        self.pose_pub = self.create_publisher(Odometry, '/ukf_pose', 10)
        self.get_logger().info('UKF node started.')

    def odom_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_odom_time is None:
            self.last_odom_time = t
            return
        dt = t - self.last_odom_time
        self.last_odom_time = t
        if dt <= 0.0 or dt > 1.0:
            return
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self.ukf.predict(v, omega, dt)
        self.publish_pose(msg.header.stamp)

    def scan_callback(self, msg):
        detections = cluster_scan(msg)
        if not detections:
            return
        for (r, b) in detections:
            z = np.array([r, b])
            best_idx, best_md2 = -1, float('inf')
            for i, lm in enumerate(LANDMARKS):
                md2 = self.ukf.mahalanobis_distance_sq(z, lm)
                if md2 < best_md2:
                    best_md2 = md2
                    best_idx = i
            if best_md2 > MAHALANOBIS_GATE:
                continue
            self.ukf.update(z, LANDMARKS[best_idx])
        self.publish_pose(msg.header.stamp)

    def publish_pose(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        mu = self.ukf.mu
        msg.pose.pose.position.x = float(mu[0])
        msg.pose.pose.position.y = float(mu[1])
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(float(mu[2]))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        Sigma = self.ukf.Sigma
        cov = [0.0] * 36
        cov[0]  = float(Sigma[0, 0])
        cov[1]  = float(Sigma[0, 1])
        cov[5]  = float(Sigma[0, 2])
        cov[6]  = float(Sigma[1, 0])
        cov[7]  = float(Sigma[1, 1])
        cov[11] = float(Sigma[1, 2])
        cov[30] = float(Sigma[2, 0])
        cov[31] = float(Sigma[2, 1])
        cov[35] = float(Sigma[2, 2])
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = UKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


main()
