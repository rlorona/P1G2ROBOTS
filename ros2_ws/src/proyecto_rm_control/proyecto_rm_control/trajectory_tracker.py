#!/usr/bin/env python3
"""Ackermann trajectory tracking controller for Proyecto_RM.

The node generates a smooth closed path (figure-eight or oval), tracks it with
a pure-pursuit law, converts the requested curvature into Ackermann steering
angles and rear-wheel speeds, publishes the low-level commands to ros2_control,
and estimates odometry from the rear-wheel encoders.

Coordinate frames
-----------------
- `odom`: fixed at the initial robot pose.  The reference path is expressed in
  this frame and starts at (0, 0) with heading +X.
- `base_footprint`: robot frame used by the controller (+X forward).
"""

from __future__ import annotations

import csv
import math
import os
import signal
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class TrajectoryTracker(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_tracker")

        # ---------------- parameters ----------------
        self.wheelbase = float(self.declare_parameter("wheelbase", 0.160).value)
        self.track_width = float(self.declare_parameter("track_width", 0.202).value)
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.037).value)
        self.max_steering = float(
            self.declare_parameter("max_steering_angle", 0.5235987756).value
        )
        self.max_speed = float(self.declare_parameter("max_speed", 0.40).value)
        self.min_speed = float(self.declare_parameter("min_speed", 0.05).value)
        self.max_accel = float(self.declare_parameter("max_accel", 0.5).value)
        self.max_steering_rate = float(
            self.declare_parameter("max_steering_rate", 1.5).value
        )
        self.max_lateral_accel = float(
            self.declare_parameter("max_lateral_accel", 0.5).value
        )
        self.lookahead_min = float(self.declare_parameter("lookahead_min", 0.22).value)
        self.lookahead_gain = float(self.declare_parameter("lookahead_gain", 0.20).value)
        self.lookahead_max = float(self.declare_parameter("lookahead_max", 0.50).value)
        self.control_rate = float(self.declare_parameter("control_rate", 50.0).value)
        self.trajectory_type = str(
            self.declare_parameter("trajectory_type", "figure8").value
        )
        self.figure8_a = float(self.declare_parameter("figure8_a", 1.8).value)
        self.figure8_b = float(self.declare_parameter("figure8_b", 1.44).value)
        self.oval_a = float(self.declare_parameter("oval_a", 1.5).value)
        self.oval_b = float(self.declare_parameter("oval_b", 1.0).value)
        self.num_laps = int(self.declare_parameter("num_laps", 3).value)
        self.start_delay = float(self.declare_parameter("start_delay", 2.0).value)
        self.stop_delay = float(self.declare_parameter("stop_delay", 2.0).value)

        self.steering_topic = str(
            self.declare_parameter(
                "steering_command_topic", "/steering_controller/commands"
            ).value
        )
        self.drive_topic = str(
            self.declare_parameter(
                "drive_command_topic", "/rear_drive_controller/commands"
            ).value
        )
        self.joint_states_topic = str(
            self.declare_parameter("joint_states_topic", "/joint_states").value
        )
        self.imu_topic = str(self.declare_parameter("imu_topic", "/imu").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/odom").value)
        self.ground_truth_topic = str(
            self.declare_parameter("ground_truth_topic", "/ground_truth/odometry").value
        )
        self.reference_path_topic = str(
            self.declare_parameter("reference_path_topic", "/reference_path").value
        )
        self.actual_path_topic = str(
            self.declare_parameter("actual_path_topic", "/actual_path").value
        )

        self.log_csv = bool(self.declare_parameter("log_csv", True).value)
        self.results_dir = str(
            self.declare_parameter("results_dir", "~/.ros/proyecto_rm").value
        )
        self.publish_paths = bool(
            self.declare_parameter("publish_paths", True).value
        )

        # ---------------- path ----------------
        self.path = self._build_path()
        self.path_length = float(
            np.sum(
                np.linalg.norm(
                    np.diff(np.vstack([self.path, self.path[0]]), axis=0), axis=1
                )
            )
        )
        self.kappa_max = 1.0 / (
            self.wheelbase / math.tan(self.max_steering) + 0.5 * self.track_width
        )

        # ---------------- state ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_est = 0.0
        self.yaw_rate_est = 0.0
        self.v_cmd = 0.0
        self.delta_left_cmd = 0.0
        self.delta_right_cmd = 0.0
        self.last_idx = 0
        self.laps = 0
        self.last_time = None
        self.start_time = None
        self.finish_time = None
        self.finished = False
        self.last_steering = (0.0, 0.0)

        self.joint_positions: dict[str, float] = {}
        self.joint_velocities: dict[str, float] = {}
        self.have_joint_states = False
        self.imu_wz = None
        self.imu_stamp = None
        self.gt = None
        self.rows = []
        self.saved = False

        self.actual_path = []

        # ---------------- ROS interfaces ----------------
        self.steering_pub = self.create_publisher(Float64MultiArray, self.steering_topic, 10)
        self.drive_pub = self.create_publisher(Float64MultiArray, self.drive_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.reference_path_pub = self.create_publisher(
            PathMsg, self.reference_path_topic, 10
        )
        self.actual_path_pub = self.create_publisher(PathMsg, self.actual_path_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(JointState, self.joint_states_topic, self._joint_cb, 20)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 50)
        self.create_subscription(
            Odometry, self.ground_truth_topic, self._ground_truth_cb, 20
        )

        self.create_timer(1.0 / max(self.control_rate, 1.0), self._control_loop)
        self.create_timer(5.0, self._publish_reference_path)
        self.create_timer(1.0, self._publish_actual_path)

        self.get_logger().info(
            f"Trajectory '{self.trajectory_type}': {len(self.path)} points, "
            f"length {self.path_length:.3f} m, kappa_max {self.kappa_max:.3f} 1/m"
        )
        self.get_logger().info(
            f"Subscribing to {self.joint_states_topic} and {self.ground_truth_topic}; "
            f"publishing to {self.steering_topic} and {self.drive_topic}"
        )

    # ------------------------------------------------------------------ path
    def _build_path(self) -> np.ndarray:
        if self.trajectory_type == "oval":
            a, b = self.oval_a, self.oval_b
            t = np.linspace(0.0, 2.0 * math.pi, 2000, endpoint=False)
            x_raw = a * np.cos(t)
            y_raw = b * np.sin(t)
            t0 = 0.5 * math.pi
            x0 = a * math.cos(t0)
            y0 = b * math.sin(t0)
            vx0 = -a * math.sin(t0)
            vy0 = b * math.cos(t0)
        else:
            a, b = self.figure8_a, self.figure8_b
            t = np.linspace(0.0, 2.0 * math.pi, 2000, endpoint=False)
            x_raw = a * np.cos(t)
            y_raw = b * np.sin(t) * np.cos(t)
            # Start at the self-intersection (t = pi/2), where curvature is zero.
            t0 = 0.5 * math.pi
            x0 = a * math.cos(t0)
            y0 = b * math.sin(t0) * math.cos(t0)
            vx0 = -a * math.sin(t0)
            vy0 = b * (math.cos(t0) ** 2 - math.sin(t0) ** 2)

        # Rotate and translate so the path starts at (0, 0) with tangent +X.
        theta0 = math.atan2(vy0, vx0)
        c, s = math.cos(-theta0), math.sin(-theta0)
        x = c * (x_raw - x0) - s * (y_raw - y0)
        y = s * (x_raw - x0) + c * (y_raw - y0)
        points = np.column_stack([x, y])
        # Make index 0 coincide with the path start (the self-intersection),
        # otherwise the lap counter starts in the middle of the array.
        start = int(np.argmin(np.linalg.norm(points, axis=1)))
        points = np.roll(points, -start, axis=0)
        return self._resample_closed(points, 1000)

    @staticmethod
    def _resample_closed(points: np.ndarray, n: int) -> np.ndarray:
        closed = np.vstack([points, points[0]])
        d = np.linalg.norm(np.diff(closed, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        si = np.linspace(0.0, s[-1], n, endpoint=False)
        return np.column_stack(
            [np.interp(si, s, closed[:, 0]), np.interp(si, s, closed[:, 1])]
        )

    # ------------------------------------------------------------- callbacks
    def _joint_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position
        for name, velocity in zip(msg.name, msg.velocity):
            self.joint_velocities[name] = velocity
        self.have_joint_states = True

    def _imu_cb(self, msg: Imu) -> None:
        self.imu_wz = msg.angular_velocity.z
        self.imu_stamp = self.get_clock().now()

    def _ground_truth_cb(self, msg: Odometry) -> None:
        self.gt = msg

    # ------------------------------------------------------------- helpers
    def _measured_wheel_speeds(self):
        wl = self.joint_velocities.get("rear_left_wheel_joint", 0.0)
        wr = self.joint_velocities.get("rear_right_wheel_joint", 0.0)
        return wl, wr

    def _update_odometry(self, now) -> None:
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.2:
            return

        if self.have_joint_states:
            wl, wr = self._measured_wheel_speeds()
        else:
            wl = self.v_cmd / self.wheel_radius
            wr = self.v_cmd / self.wheel_radius

        self.v_est = self.wheel_radius * 0.5 * (wl + wr)

        # Prefer the IMU yaw rate when it is fresh; fall back to the steering
        # geometry otherwise.
        imu_ok = False
        if self.imu_wz is not None and self.imu_stamp is not None:
            age = (now - self.imu_stamp).nanoseconds * 1e-9
            if 0.0 <= age < 0.25:
                self.yaw_rate_est = self.imu_wz
                imu_ok = True

        if not imu_ok:
            # Yaw rate from the measured steering angles.  For an Ackermann
            # vehicle the rear-axle centre follows a circle of radius R, and
            # each front wheel satisfies tan(delta) = L / (R -/+ t/2).
            dl = self.joint_positions.get("front_left_steering_joint")
            dr = self.joint_positions.get("front_right_steering_joint")
            if dl is not None and dr is not None and (
                abs(math.tan(dl)) > 1e-3 or abs(math.tan(dr)) > 1e-3
            ):
                radii = []
                if abs(math.tan(dl)) > 1e-3:
                    radii.append(
                        self.wheelbase / math.tan(dl) + 0.5 * self.track_width
                    )
                if abs(math.tan(dr)) > 1e-3:
                    radii.append(
                        self.wheelbase / math.tan(dr) - 0.5 * self.track_width
                    )
                radius = float(np.mean(radii))
                self.yaw_rate_est = (
                    self.v_est / radius if abs(radius) > 1e-3 else 0.0
                )
            else:
                self.yaw_rate_est = 0.0

        # `self.v_est` is the rear-axle longitudinal speed.  The controlled
        # frame (base_footprint) is at the mid-point between the axles, so its
        # body-frame velocity is (v, omega * L/2): the rotation of the chassis
        # adds a lateral component that must be integrated as well.
        w = self.yaw_rate_est
        a = 0.5 * self.wheelbase
        mid_yaw = self.yaw + 0.5 * w * dt
        self.x += (self.v_est * math.cos(mid_yaw) - w * a * math.sin(mid_yaw)) * dt
        self.y += (self.v_est * math.sin(mid_yaw) + w * a * math.cos(mid_yaw)) * dt
        self.yaw = wrap_angle(self.yaw + w * dt)

    def _closest_index(self) -> int:
        n = len(self.path)
        if self.last_idx is None:
            candidates = np.arange(n)
        else:
            offsets = np.arange(-25, 250)
            candidates = (self.last_idx + offsets) % n
        d = np.linalg.norm(self.path[candidates] - np.array([self.x, self.y]), axis=1)
        return int(candidates[int(np.argmin(d))])

    def _lookahead_point(self, idx: int, lookahead: float) -> np.ndarray:
        n = len(self.path)
        p = np.array([self.x, self.y])
        best = self.path[idx]
        for k in range(1, n):
            j = (idx + k) % n
            if np.linalg.norm(self.path[j] - p) >= lookahead:
                return self.path[j]
            best = self.path[j]
        return best

    # ------------------------------------------------------------- control
    def _publish_zero(self) -> None:
        self.steering_pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        self.drive_pub.publish(Float64MultiArray(data=[0.0, 0.0]))

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        self._update_odometry(now)
        self._publish_odom(now)

        if (now - self.start_time).nanoseconds * 1e-9 < self.start_delay:
            self._publish_zero()
            return

        if self.finished:
            self._publish_zero()
            self._log_row(now, 0.0, 0.0, 0.0)
            if (now - self.finish_time).nanoseconds * 1e-9 >= self.stop_delay:
                self.get_logger().info("Saving results and shutting down.")
                self.save_results()
                rclpy.shutdown()
            return

        dt = 1.0 / max(self.control_rate, 1.0)

        idx = self._closest_index()
        if self.last_idx is not None and self.last_idx > 0.85 * len(self.path) and idx < 0.15 * len(self.path):
            self.laps += 1
            self.get_logger().info(f"Lap {self.laps}/{self.num_laps} completed")
            if self.laps >= self.num_laps:
                self.finished = True
                self.finish_time = now
                self.get_logger().info("Trajectory complete; stopping the robot.")
        self.last_idx = idx

        if self.finished:
            self._publish_zero()
            return

        lookahead = float(
            np.clip(
                self.lookahead_min + self.lookahead_gain * self.v_cmd,
                self.lookahead_min,
                self.lookahead_max,
            )
        )
        target = self._lookahead_point(idx, lookahead)
        alpha = wrap_angle(
            math.atan2(target[1] - self.y, target[0] - self.x) - self.yaw
        )
        kappa_mid = 2.0 * math.sin(alpha) / max(lookahead, 1e-6)
        a = 0.5 * self.wheelbase
        kappa_mid_max = self.kappa_max / math.sqrt(
            1.0 + (self.kappa_max * a) ** 2
        )
        kappa_mid = float(np.clip(kappa_mid, -kappa_mid_max, kappa_mid_max))

        # The control point is base_footprint (between the axles), while the
        # steering geometry is defined at the rear axle.  Convert the
        # mid-point curvature into the equivalent rear-axle curvature.
        if abs(kappa_mid) > 1e-6:
            kappa = kappa_mid / math.sqrt(
                max(1.0 - (kappa_mid * a) ** 2, 1e-6)
            )
            kappa = float(np.clip(kappa, -self.kappa_max, self.kappa_max))
        else:
            kappa = 0.0

        if abs(kappa) < 1e-6:
            delta_left = 0.0
            delta_right = 0.0
        else:
            radius = 1.0 / kappa
            delta_left = math.atan(
                self.wheelbase / (radius - 0.5 * self.track_width)
            )
            delta_right = math.atan(
                self.wheelbase / (radius + 0.5 * self.track_width)
            )
            delta_left = float(np.clip(delta_left, -self.max_steering, self.max_steering))
            delta_right = float(
                np.clip(delta_right, -self.max_steering, self.max_steering)
            )

        # steering rate limit
        max_delta = self.max_steering_rate * dt
        delta_left = float(
            np.clip(delta_left, self.last_steering[0] - max_delta, self.last_steering[0] + max_delta)
        )
        delta_right = float(
            np.clip(delta_right, self.last_steering[1] - max_delta, self.last_steering[1] + max_delta)
        )
        self.last_steering = (delta_left, delta_right)

        # speed scheduling: lateral acceleration and longitudinal acceleration
        v_curve = math.sqrt(
            self.max_lateral_accel / max(abs(kappa_mid), 1e-6)
        )
        v_target = max(self.min_speed, min(self.max_speed, v_curve))
        self.v_cmd = float(
            np.clip(
                v_target,
                self.v_cmd - self.max_accel * dt,
                self.v_cmd + self.max_accel * dt,
            )
        )

        # Rear-axle speed that makes the mid-point travel at v_cmd.
        v_rear = self.v_cmd / math.sqrt(1.0 + (kappa * a) ** 2)
        v_left = v_rear * (1.0 - kappa * 0.5 * self.track_width)
        v_right = v_rear * (1.0 + kappa * 0.5 * self.track_width)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        self.steering_pub.publish(Float64MultiArray(data=[delta_left, delta_right]))
        self.drive_pub.publish(Float64MultiArray(data=[omega_left, omega_right]))

        self.actual_path.append((self.x, self.y, self.yaw))
        self._log_row(now, kappa_mid, delta_left, delta_right, omega_left, omega_right)

    # ------------------------------------------------------------- outputs
    def _publish_odom(self, now) -> None:
        qz, qw = quat_from_yaw(self.yaw)[2:]
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.v_est
        msg.twist.twist.angular.z = self.yaw_rate_est
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.02
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[35] = 0.02
        self.odom_pub.publish(msg)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def _publish_reference_path(self) -> None:
        if not self.publish_paths:
            return
        from geometry_msgs.msg import PoseStamped

        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        n = len(self.path)
        for i in range(n):
            p = self.path[i]
            q = self.path[(i + 1) % n]
            yaw = math.atan2(q[1] - p[1], q[0] - p[0])
            qz, qw = quat_from_yaw(yaw)[2:]
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)
        self.reference_path_pub.publish(msg)

    def _publish_actual_path(self) -> None:
        if not self.publish_paths or not self.actual_path:
            return
        from geometry_msgs.msg import PoseStamped

        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        for x, y, yaw in self.actual_path[-4000:]:
            qz, qw = quat_from_yaw(yaw)[2:]
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            msg.poses.append(ps)
        self.actual_path_pub.publish(msg)

    # ------------------------------------------------------------- logging
    def _log_row(self, now, kappa, delta_left, delta_right, omega_left=0.0, omega_right=0.0) -> None:
        if not self.log_csv:
            return
        ref = self.path[self.last_idx]
        cte = float(np.linalg.norm(np.array([self.x, self.y]) - ref))
        gt_x = gt_y = gt_yaw = cte_gt = float("nan")
        if self.gt is not None:
            gt_x = self.gt.pose.pose.position.x
            gt_y = self.gt.pose.pose.position.y
            q = self.gt.pose.pose.orientation
            gt_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            cte_gt = float(
                np.linalg.norm(np.array([gt_x, gt_y]) - self.path[self.last_idx])
            )
        self.rows.append(
            [
                now.nanoseconds * 1e-9,
                self.x,
                self.y,
                self.yaw,
                gt_x,
                gt_y,
                gt_yaw,
                float(ref[0]),
                float(ref[1]),
                cte,
                cte_gt,
                kappa,
                self.v_cmd,
                delta_left,
                delta_right,
                omega_left,
                omega_right,
                self.joint_velocities.get("rear_left_wheel_joint", 0.0),
                self.joint_velocities.get("rear_right_wheel_joint", 0.0),
                self.joint_positions.get("front_left_steering_joint", 0.0),
                self.joint_positions.get("front_right_steering_joint", 0.0),
            ]
        )

    def save_results(self) -> None:
        if not self.rows or self.saved:
            return
        self.saved = True
        out_dir = Path(os.path.expanduser(self.results_dir))
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / "results.csv"
        tmp_path = out_dir / "results.csv.tmp"
        with open(tmp_path, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(
                [
                    "time",
                    "odom_x",
                    "odom_y",
                    "odom_yaw",
                    "gt_x",
                    "gt_y",
                    "gt_yaw",
                    "ref_x",
                    "ref_y",
                    "cross_track_error",
                    "cross_track_error_gt",
                    "kappa",
                    "v_cmd",
                    "delta_left",
                    "delta_right",
                    "omega_left_cmd",
                    "omega_right_cmd",
                    "omega_left_meas",
                    "omega_right_meas",
                    "delta_left_meas",
                    "delta_right_meas",
                ]
            )
            writer.writerows(self.rows)
        os.replace(tmp_path, csv_path)

        cte = np.array([r[9] for r in self.rows], dtype=float)
        cte = cte[np.isfinite(cte)]
        cte_gt = np.array([r[10] for r in self.rows], dtype=float)
        cte_gt = cte_gt[np.isfinite(cte_gt)]
        odom = np.array([[r[1], r[2]] for r in self.rows], dtype=float)
        gt = np.array([[r[4], r[5]] for r in self.rows], dtype=float)
        mask = np.isfinite(gt).all(axis=1)
        drift = np.linalg.norm(odom[mask] - gt[mask], axis=1) if mask.any() else np.array([])

        summary = []
        summary.append(f"trajectory: {self.trajectory_type}")
        summary.append(f"path_length_m: {self.path_length:.4f}")
        summary.append(f"laps_completed: {self.laps}")
        if cte.size:
            summary.append(f"cross_track_rmse_m: {float(np.sqrt(np.mean(cte**2))):.6f}")
            summary.append(f"cross_track_mean_m: {float(np.mean(cte)):.6f}")
            summary.append(f"cross_track_max_m: {float(np.max(cte)):.6f}")
        if cte_gt.size:
            summary.append(f"cross_track_gt_rmse_m: {float(np.sqrt(np.mean(cte_gt**2))):.6f}")
            summary.append(f"cross_track_gt_mean_m: {float(np.mean(cte_gt)):.6f}")
            summary.append(f"cross_track_gt_max_m: {float(np.max(cte_gt)):.6f}")
        if drift.size:
            summary.append(f"odom_drift_rmse_m: {float(np.sqrt(np.mean(drift**2))):.6f}")
            summary.append(f"odom_drift_final_m: {float(drift[-1]):.6f}")
        summary.append(f"samples: {len(self.rows)}")
        (out_dir / "summary.txt").write_text("\n".join(summary) + "\n")
        self.get_logger().info("Results written to %s" % out_dir)
        for line in summary:
            self.get_logger().info(line)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ignore repeated SIGINT/SIGTERM while the results are being flushed,
        # otherwise the CSV file can be truncated by a second signal.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.save_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
