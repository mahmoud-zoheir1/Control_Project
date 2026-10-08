"""
Teleoperation Bridge Node

Milestone 3:
- Open-loop throttle and steering mapping.
- Safety watchdog.

Milestone 4:
- Closed-loop cruise control using a longitudinal PID.
- Actual velocity feedback from /state.
"""

import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from bicycle_control.longitudinal_pid import PIDLongitudinalController


class TeleopBridge(Node):

    def __init__(self):
        super().__init__('teleop_bridge')

        self.get_logger().info('Teleoperation Bridge Node Initialized')

        # Parameters
        self.declare_parameter('max_linear_vel', 5.0)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('max_steer_rad', 0.610865)
        self.declare_parameter('auto_zero_timeout', 0.5)
        self.declare_parameter('use_cruise_control', False)

        self.max_linear_vel = float(
            self.get_parameter('max_linear_vel').value
        )

        self.max_angular_vel = float(
            self.get_parameter('max_angular_vel').value
        )

        self.max_steer_rad = float(
            self.get_parameter('max_steer_rad').value
        )

        self.auto_zero_timeout = float(
            self.get_parameter('auto_zero_timeout').value
        )

        self.use_cruise_control = bool(
            self.get_parameter('use_cruise_control').value
        )

        # Publishers
        self.throttle_pub = self.create_publisher(
            Float32, '/throttle', 10
        )

        self.steer_pub = self.create_publisher(
            Float32, '/steer', 10
        )

        # Keyboard commands subscriber
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Vehicle states
        self.current_throttle = 0.0
        self.current_steer = 0.0
        self.target_vel = 0.0
        self.current_vel = 0.0

        self.last_cmd_time = self.get_clock().now()

        # Milestone 4: PID controller
        self.pid_controller = PIDLongitudinalController(
            kp=1.0,
            ki=0.2,
            kd=0.05,
            dt=0.1
        )

        # Milestone 4: Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/state',
            self.odom_callback,
            10
        )

        # Publish commands at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.publish_commands
        )

        if self.use_cruise_control:
            self.get_logger().info('Cruise Control ENABLED')
        else:
            self.get_logger().info('Open-Loop Teleoperation ENABLED')

    # =========================================================
    # Milestone 4: Actual velocity feedback
    # =========================================================

    def odom_callback(self, msg: Odometry):
        """Read vehicle forward velocity from odometry."""

        self.current_vel = float(
            msg.twist.twist.linear.x
        )

    # =========================================================
    # Milestone 3 + 4: Keyboard commands
    # =========================================================

    def cmd_callback(self, msg: Twist):
        """Receive target velocity and steering commands."""

        # Target speed for cruise control
        self.target_vel = float(
            np.clip(
                msg.linear.x,
                -self.max_linear_vel,
                self.max_linear_vel
            )
        )

        # Milestone 3: Open-loop throttle
        if not self.use_cruise_control:

            throttle = msg.linear.x / self.max_linear_vel

            throttle = np.clip(
                throttle,
                -1.0,
                1.0
            )

            self.current_throttle = float(throttle)

        # Steering mapping (both modes)
        steer = (
            msg.angular.z / self.max_angular_vel
        ) * self.max_steer_rad

        steer = np.clip(
            steer,
            -self.max_steer_rad,
            self.max_steer_rad
        )

        self.current_steer = float(steer)

        # Update watchdog
        self.last_cmd_time = self.get_clock().now()

    # =========================================================
    # Milestone 3 + 4: Publish throttle and steering
    # =========================================================

    def publish_commands(self):
        """Publish throttle and steering at 10 Hz."""

        current_time = self.get_clock().now()

        elapsed_time = (
            current_time - self.last_cmd_time
        ).nanoseconds / 1e9

        # Safety watchdog
        if elapsed_time > self.auto_zero_timeout:

            self.current_throttle = 0.0
            self.current_steer = 0.0
            self.target_vel = 0.0

            if self.use_cruise_control:
                self.pid_controller.reset()

            throttle = 0.0

        else:

            if self.use_cruise_control:

                # Milestone 4: Closed-loop PID
                throttle = self.pid_controller.compute(
                    self.target_vel,
                    self.current_vel
                )

            else:

                # Milestone 3: Open-loop
                throttle = self.current_throttle

        # Publish throttle
        self.throttle_pub.publish(
            Float32(data=float(throttle))
        )

        # Publish steering
        self.steer_pub.publish(
            Float32(data=float(self.current_steer))
        )


def main(args=None):

    rclpy.init(args=args)

    bridge = TeleopBridge()

    try:
        rclpy.spin(bridge)

    except KeyboardInterrupt:
        pass

    finally:
        bridge.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':      
    main()
