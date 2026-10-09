"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        # TODO: Milestone 5.3 Step 1 — Adaptive Lookahead Horizon
        # The car looks further ahead at higher speeds to plan smoother turns.
        # Implement the speed-scaled lookahead formula and clamp it to the allowed range.
        lookahead = self.kv * abs(v) + self.l_min

        lookahead = max(
            self.l_min,
            min(lookahead, self.l_max)
        )

        return lookahead

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        # TODO: Milestone 5.3 Step 2 — Target Waypoint Selection
        # This selects the goal point the car will steer toward.
        # Find the nearest waypoint on the path, then walk forward until
        # you reach one that is at least 'lookahead' meters away.
        if not path_points:
            return None, None

        # Find the nearest waypoint to the vehicle
        nearest_idx = min(
            range(len(path_points)),
            key=lambda i: math.hypot(
                path_points[i][0] - x,
                path_points[i][1] - y
            )
        )

        # Search forward along the path
        for step in range(len(path_points)):
            idx = (nearest_idx + step) % len(path_points)

            px, py = path_points[idx][0], path_points[idx][1]
            distance = math.hypot(px - x, py - y)

            if distance >= lookahead:
                return idx, path_points[idx]

        # Fallback if no waypoint satisfies the lookahead
        return nearest_idx, path_points[nearest_idx]

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        # TODO: Milestone 5.3 Steps 3 & 4 — Coordinate Transformation & Arc Law
        # This is the core of Pure Pursuit: transform the target into the vehicle's
        # local frame, then use the arc geometry formula to compute the steering angle.
        # Target position relative to the vehicle
        dx = target_pt[0] - x
        dy = target_pt[1] - y

        # Transform target into vehicle coordinates
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # Actual distance to the target
        ld = max(math.hypot(dx, dy), 1e-6)

        # Pure Pursuit curvature
        curvature = 2.0 * local_y / (ld * ld)

        # Convert curvature to steering angle
        steer = math.atan(self.L * curvature)

        # Limit steering to +/- 35 degrees
        steer = max(
            -self.max_steer_rad,
            min(steer, self.max_steer_rad)
        )

        return steer
