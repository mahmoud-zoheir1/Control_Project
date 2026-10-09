"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculate curvature-limited target speed."""

        if abs(kappa) < 1e-6:
            target_speed = self.max_speed
        else:
            target_speed = math.sqrt(
                self.max_lat_accel / abs(kappa)
            )

        target_speed = min(target_speed, self.max_speed)

        if fallback_speed is not None:
            target_speed = min(target_speed, fallback_speed)

        return max(0.0, target_speed)