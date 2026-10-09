
"""
Lap Analyzer Node:
Performance evaluation, real-time telemetry, lap timing,
and RViz HUD visualization.
"""

import json
import math
import numpy as np
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class LapAnalyzer(Node):

    def __init__(self):
        super().__init__('lap_analyzer')
        self.get_logger().info('Initializing Lap Analyzer Node...')

        # Subscriptions
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10
        )
        self.state_sub = self.create_subscription(
            Odometry, '/state', self.state_callback, 10
        )

        # Publishers
        self.metrics_pub = self.create_publisher(
            String, '/lap/metrics', 10
        )
        self.viz_pub = self.create_publisher(
            MarkerArray, '/lap/visualization', 10
        )

        # Real-time telemetry publishers
        self.cte_pub = self.create_publisher(
            Float32, '/telemetry/cte', 10
        )
        self.speed_pub = self.create_publisher(
            Float32, '/telemetry/speed', 10
        )
        self.heading_err_pub = self.create_publisher(
            Float32, '/telemetry/heading_err_deg', 10
        )
        self.lap_time_pub = self.create_publisher(
            Float32, '/telemetry/lap_time', 10
        )

        # Path storage
        self.path_points = []
        self.path_cum_dist = []
        self.track_length = 0.0
        self.path_received = False

        # State and timing
        self.start_sim_time = None
        self.last_state_time = None
        self.lap_start_time = None

        # Lap tracking
        self.lap_count = 0
        self.last_s = 0.0
        self.total_distance = 0.0
        self.last_xy = None

        # Lap times
        self.current_lap_time = 0.0
        self.last_lap_time = None
        self.best_lap_time = None
        self.lap_times = []

        # Per-lap statistics
        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

        # Global statistics
        self.global_ctes = []
        self.global_max_speed = 0.0

        # Current telemetry
        self.current_cte = 0.0
        self.current_heading_err = 0.0
        self.current_speed = 0.0
        self.proj_xy = (0.0, 0.0)

        # Publish at 10 Hz
        self.timer = self.create_timer(
            0.1, self.publish_telemetry
        )

    def path_callback(self, msg: Path):
        """Load reference path and calculate cumulative distance."""

        if self.path_received and len(msg.poses) == len(self.path_points):
            return

        pts = []

        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y

            qz = p.pose.orientation.z
            qw = p.pose.orientation.w

            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))

        if len(pts) < 2:
            return

        self.path_points = pts

        cum = [0.0]

        for i in range(1, len(pts)):
            d = math.hypot(
                pts[i][0] - pts[i - 1][0],
                pts[i][1] - pts[i - 1][1]
            )
            cum.append(cum[-1] + d)

        self.path_cum_dist = cum
        self.track_length = cum[-1]
        self.path_received = True

        self.get_logger().info(
            f"Lap Analyzer: Loaded path with {len(pts)} waypoints, "
            f"perimeter: {self.track_length:.2f} m"
        )

    def state_callback(self, msg: Odometry):
        """Update vehicle state, errors, and lap timing."""

        now_sec = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        if self.start_sim_time is None:
            self.start_sim_time = now_sec
            self.lap_start_time = now_sec

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x

        self.current_speed = v
        self.global_max_speed = max(
            self.global_max_speed, v
        )

        # Total traveled distance
        if self.last_xy is not None:
            step_d = math.hypot(
                x - self.last_xy[0],
                y - self.last_xy[1]
            )
            self.total_distance += step_d

        self.last_xy = (x, y)

        if not self.path_received or len(self.path_points) < 2:
            return

        # Project vehicle position onto reference path
        proj_x, proj_y, s, cte, heading_err = (
            self.project_to_path(x, y, yaw)
        )

        self.proj_xy = (proj_x, proj_y)
        self.current_cte = cte
        self.current_heading_err = heading_err

        # Store performance measurements
        abs_cte = abs(cte)

        self.lap_ctes.append(abs_cte)
        self.lap_heading_errors.append(abs(heading_err))
        self.lap_speeds.append(v)
        self.global_ctes.append(abs_cte)

        self.current_lap_time = (
            now_sec - self.lap_start_time
        )

        # Detect completion when path progress wraps
        if self.track_length > 5.0 and v > 0.1:

            crossed_finish = (
                self.last_s > 0.75 * self.track_length
                and s < 0.25 * self.track_length
            )

            if crossed_finish:

                ds_total = (
                    self.track_length - self.last_s + s
                )

                previous_time = (
                    self.last_state_time
                    if self.last_state_time is not None
                    else now_sec
                )

                dt_step = max(
                    now_sec - previous_time, 1e-4
                )

                frac = (
                    (self.track_length - self.last_s)
                    / max(ds_total, 1e-4)
                )

                t_crossing = (
                    previous_time + frac * dt_step
                )

                lap_duration = (
                    t_crossing - self.lap_start_time
                )

                self.record_lap_completion(
                    lap_duration, now_sec
                )

                self.lap_start_time = t_crossing
                self.current_lap_time = max(
                    0.0, now_sec - t_crossing
                )

        self.last_s = s
        self.last_state_time = now_sec

    def project_to_path(self, x, y, yaw):
        """Calculate projected position, signed CTE and heading error."""

        pts = self.path_points
        n = len(pts)

        min_dist_sq = float('inf')
        nearest_idx = 0

        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y

            d_sq = dx * dx + dy * dy

            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                nearest_idx = i

        best_dist = float('inf')

        best_proj = (
            pts[nearest_idx][0],
            pts[nearest_idx][1]
        )

        best_s = self.path_cum_dist[nearest_idx]
        best_seg_yaw = pts[nearest_idx][2]
        best_signed_cte = 0.0

        candidate_segments = [
            ((nearest_idx - 1) % n, nearest_idx),
            (nearest_idx, (nearest_idx + 1) % n)
        ]

        for prev_i, next_i in candidate_segments:

            x1, y1, _ = pts[prev_i]
            x2, y2, _ = pts[next_i]

            dx = x2 - x1
            dy = y2 - y1

            seg_len_sq = dx * dx + dy * dy

            if seg_len_sq < 1e-6:
                continue

            t = max(
                0.0,
                min(
                    1.0,
                    (
                        (x - x1) * dx
                        + (y - y1) * dy
                    ) / seg_len_sq
                )
            )

            px = x1 + t * dx
            py = y1 + t * dy

            dist = math.hypot(
                x - px, y - py
            )

            if dist < best_dist:

                best_dist = dist
                best_proj = (px, py)

                best_s = (
                    self.path_cum_dist[prev_i]
                    + t * math.sqrt(seg_len_sq)
                )

                best_seg_yaw = math.atan2(dy, dx)

                cross = (
                    dx * (y - y1)
                    - dy * (x - x1)
                )

                best_signed_cte = math.copysign(
                    dist, cross
                )

        heading_err = math.atan2(
            math.sin(yaw - best_seg_yaw),
            math.cos(yaw - best_seg_yaw)
        )

        return (
            best_proj[0],
            best_proj[1],
            best_s,
            best_signed_cte,
            heading_err
        )

    def record_lap_completion(self, lap_duration, now_sec):
        """Calculate and log performance statistics for a completed lap."""

        self.lap_count += 1
        self.last_lap_time = lap_duration
        self.lap_times.append(lap_duration)

        if (
            self.best_lap_time is None
            or lap_duration < self.best_lap_time
        ):
            self.best_lap_time = lap_duration

        ctes = np.array(
            self.lap_ctes, dtype=float
        )

        speeds = np.array(
            self.lap_speeds, dtype=float
        )

        mean_cte = (
            float(np.mean(ctes))
            if ctes.size else 0.0
        )

        max_cte = (
            float(np.max(ctes))
            if ctes.size else 0.0
        )

        rms_cte = (
            float(np.sqrt(np.mean(ctes ** 2)))
            if ctes.size else 0.0
        )

        mean_speed = (
            float(np.mean(speeds))
            if speeds.size else 0.0
        )

        max_speed = (
            float(np.max(speeds))
            if speeds.size else 0.0
        )

        self.get_logger().info(
            f"\n{'=' * 45}\n"
            f"LAP {self.lap_count} COMPLETED\n"
            f"Lap Time:   {lap_duration:.2f} s\n"
            f"Best Lap:   {self.best_lap_time:.2f} s\n"
            f"Mean CTE:   {mean_cte:.3f} m\n"
            f"RMS CTE:    {rms_cte:.3f} m\n"
            f"Max CTE:    {max_cte:.3f} m\n"
            f"Mean Speed: {mean_speed:.2f} m/s\n"
            f"Max Speed:  {max_speed:.2f} m/s\n"
            f"Distance:   {self.total_distance:.2f} m\n"
            f"{'=' * 45}"
        )

        # Reset per-lap statistics
        self.lap_ctes.clear()
        self.lap_heading_errors.clear()
        self.lap_speeds.clear()

    def publish_telemetry(self):
        """Publish telemetry and visualization markers at 10 Hz."""

        # Numerical telemetry
        self.cte_pub.publish(
            Float32(data=float(self.current_cte))
        )

        self.speed_pub.publish(
            Float32(data=float(self.current_speed))
        )

        self.heading_err_pub.publish(
            Float32(
                data=float(
                    math.degrees(self.current_heading_err)
                )
            )
        )

        self.lap_time_pub.publish(
            Float32(data=float(self.current_lap_time))
        )

        # Current-lap RMS cross-track error
        ctes = np.array(
            self.lap_ctes, dtype=float
        )

        rms_cte = (
            float(np.sqrt(np.mean(ctes ** 2)))
            if ctes.size else 0.0
        )

        # JSON telemetry
        telemetry = {
            'lap': self.lap_count,
            'current_lap_time': self.current_lap_time,
            'last_lap_time': self.last_lap_time,
            'best_lap_time': self.best_lap_time,
            'speed': self.current_speed,
            'current_cte': self.current_cte,
            'rms_cte': rms_cte,
            'heading_err_deg': math.degrees(
                self.current_heading_err
            )
        }

        self.metrics_pub.publish(
            String(data=json.dumps(telemetry))
        )

        self.publish_rviz_markers(telemetry)

    def publish_rviz_markers(self, telemetry=None):
        """Publish start gate, CTE whisker and telemetry HUD."""

        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Marker 1: Start / finish gate
        if self.path_points:

            p0 = self.path_points[0]

            gate = Marker()
            gate.header.frame_id = 'map'
            gate.header.stamp = now

            gate.ns = 'start_gate'
            gate.id = 0
            gate.type = Marker.CYLINDER
            gate.action = Marker.ADD

            gate.pose.position.x = p0[0]
            gate.pose.position.y = p0[1]
            gate.pose.position.z = 0.5
            gate.pose.orientation.w = 1.0

            gate.scale.x = 0.1
            gate.scale.y = 1.2
            gate.scale.z = 1.0

            gate.color.r = 0.1
            gate.color.g = 0.9
            gate.color.b = 0.2
            gate.color.a = 0.7

            ma.markers.append(gate)

        # Marker 2: Cross-track error whisker
        if (
            self.last_xy is not None
            and self.path_received
        ):

            whisker = Marker()
            whisker.header.frame_id = 'map'
            whisker.header.stamp = now

            whisker.ns = 'cte_whisker'
            whisker.id = 1
            whisker.type = Marker.LINE_STRIP
            whisker.action = Marker.ADD

            whisker.pose.orientation.w = 1.0
            whisker.scale.x = 0.06

            p_car = Point()
            p_car.x = float(self.last_xy[0])
            p_car.y = float(self.last_xy[1])
            p_car.z = 0.2

            p_path = Point()
            p_path.x = float(self.proj_xy[0])
            p_path.y = float(self.proj_xy[1])
            p_path.z = 0.2

            whisker.points = [
                p_car, p_path
            ]

            error_ratio = min(
                abs(self.current_cte) / 1.0,
                1.0
            )

            whisker.color.r = error_ratio
            whisker.color.g = 1.0 - error_ratio
            whisker.color.b = 0.0
            whisker.color.a = 1.0

            ma.markers.append(whisker)

        # Marker 3: Floating telemetry HUD
        if (
            telemetry is not None
            and self.last_xy is not None
        ):

            hud = Marker()
            hud.header.frame_id = 'map'
            hud.header.stamp = now

            hud.ns = 'telemetry_hud'
            hud.id = 2
            hud.type = Marker.TEXT_VIEW_FACING
            hud.action = Marker.ADD

            hud.pose.position.x = float(
                self.last_xy[0]
            )
            hud.pose.position.y = float(
                self.last_xy[1]
            )
            hud.pose.position.z = 2.5

            hud.pose.orientation.w = 1.0
            hud.scale.z = 0.5

            hud.color.r = 1.0
            hud.color.g = 1.0
            hud.color.b = 1.0
            hud.color.a = 1.0

            best = telemetry['best_lap_time']

            best_text = (
                f'{best:.2f}s'
                if best is not None
                else 'N/A'
            )

            hud.text = (
                f"Lap: {telemetry['lap'] + 1}\n"
                f"Time: {telemetry['current_lap_time']:.1f}s\n"
                f"Speed: {telemetry['speed']:.2f} m/s\n"
                f"CTE: {telemetry['current_cte']:.3f} m\n"
                f"Best: {best_text}"
            )

            ma.markers.append(hud)

        self.viz_pub.publish(ma)


def main(args=None):

    rclpy.init(args=args)
    analyzer = LapAnalyzer()

    try:
        rclpy.spin(analyzer)

    except KeyboardInterrupt:
        pass

    finally:
        analyzer.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
