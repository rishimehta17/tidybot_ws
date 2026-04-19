#!/usr/bin/env python3
"""
TidyBot waypoint navigator.

This node drives the robot through hard-coded waypoints, logs progress,
and optionally triggers simple arm actions at selected waypoints.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion

# Waypoints can be one of the following:
#   (x, y, label)
#   (x, y, label, action)
#   (x, y, label, yaw_deg, action)
WAYPOINTS = [
    (0.0, -2.0, 'Room1 – south area'),
    (1.0, 1.0, 'Room1 – west side', 'PICK_UP_STOW'),
    (-0.2, -0.2, 'Room1 – north area'),
    (-0.6, -0.6, 'Room1 – east, approaching door', 'DROP'),
    (2.5, 0.0, 'DOORWAY – crossing into Room 2'),
    (3.7, 0.6, 'DOORWAY – crossing into Room 2'),
    (3.5, -2.0, 'Room2 – southwest corner'),
    (5.5, -1.5, 'Room2 – pre-approach south'),
    (6.1, -1.45, 'Room2 – approach south'),
    (6.9, -2.7, 'Room2 – approach south'),
    (5.9, -1.8, 'Room2 – pre-approach south'),
    (5.7, -1.2, 'Room2 – rise 2'),
    (6.5, -1.5, 'Room2 – northeast approach'),
    (7.0, 1.0, 'Room2 – top right'),
    (5.85, 2.35, 'Room2 – top right'),
    (3.5, 2.5, 'Room2 – back toward door'),
    (3.0, 0.0, 'Room2 – back toward door'),
    (2.0, 0.0, 'DOORWAY – crossing back into Room 1'),
    (-0.1, -0.1, 'Room1 – home position'),
]

ARM_STOW = (1.2, 1.57, -1.0, 0.0)
ARM_PICK = (-0.5, 1.0, -1.0, 0.0)
ARM_UP = (0.5, 1.0, -0.5, 0.5)

DRIVE_ANG_KP, DRIVE_ANG_KI, DRIVE_ANG_KD = 1.6, 0.008, 0.10
DRIVE_LIN_KP, DRIVE_LIN_KI, DRIVE_LIN_KD = 0.5, 0.004, 0.06
DRIVE_ANG_MAX = 1.2
DRIVE_LIN_MAX = 0.45

GOAL_TOL = 0.3
YAW_TOL = math.radians(5.0)
HEADING_FREEZE_DIST = 0.35
BLEND_RADIUS = 1.0
LOG_INTERVAL = 1.0


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    """Simple PID controller with output saturation."""

    def __init__(self, kp, ki, kd, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t = None

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t = None

    def compute(self, error, now):
        dt = (now - self._prev_t) if self._prev_t else 0.05
        dt = max(dt, 1e-6)
        self._integral += error * dt
        deriv = (error - self._prev_err) / dt
        out = self.kp * error + self.ki * self._integral + self.kd * deriv
        self._prev_err = error
        self._prev_t = now
        return max(-self.out_max, min(self.out_max, out))


class Navigator(Node):
    """Waypoint-based navigator with simple arm action hooks."""

    def __init__(self):
        super().__init__('tidybot_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10,
        )
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False

        self._total_distance = 0.0
        self._prev_log_x = None
        self._prev_log_y = None
        self._rooms_visited = set()
        self._last_log_time = 0.0
        self._wp_reached_times = []

        self.ang_pid = PID(
            DRIVE_ANG_KP,
            DRIVE_ANG_KI,
            DRIVE_ANG_KD,
            DRIVE_ANG_MAX,
        )
        self.lin_pid = PID(
            DRIVE_LIN_KP,
            DRIVE_LIN_KI,
            DRIVE_LIN_KD,
            DRIVE_LIN_MAX,
        )

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_ready = True

        if self._prev_log_x is not None:
            dx = self.x - self._prev_log_x
            dy = self.y - self._prev_log_y
            self._total_distance += math.hypot(dx, dy)

        self._prev_log_x = self.x
        self._prev_log_y = self.y

        if self.x < 2.5:
            self._rooms_visited.add('Room 1')
        else:
            self._rooms_visited.add('Room 2')

    def _send_arm(self, preset):
        shoulder, elbow, wrist, gripper = preset
        msg = Float64MultiArray()
        msg.data = [
            float(shoulder),
            float(elbow),
            float(wrist),
            float(gripper),
            float(-gripper),
            float(gripper),
            float(-gripper),
            float(shoulder),
            float(elbow),
            float(wrist),
            float(gripper),
            float(-gripper),
            float(gripper),
            float(-gripper),
        ]
        self.arm_pub.publish(msg)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _log_status(self, wp_label, dist_to_wp, now):
        if now - self._last_log_time < LOG_INTERVAL:
            return

        self._last_log_time = now
        rooms = ', '.join(sorted(self._rooms_visited)) or 'none yet'
        print(
            f"  [t={now:6.1f}s]  "
            f"pos=({self.x:+6.2f}, {self.y:+6.2f})  "
            f"yaw={math.degrees(self.yaw):+6.1f}°  "
            f"dist_to_wp={dist_to_wp:.2f}m  "
            f"total_dist={self._total_distance:.2f}m  "
            f"rooms={rooms}",
            flush=True,
        )

    def _go_to(self, tx, ty, label, t0, target_yaw=None):
        """Drive to a single waypoint with optional yaw alignment."""
        self.ang_pid.reset()
        self.lin_pid.reset()

        print(f"\n→ WAYPOINT: {label}  target=({tx}, {ty})")
        frozen_heading_err = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic() - t0

            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)

            self._log_status(label, dist, now)

            if dist < GOAL_TOL:
                if target_yaw is not None:
                    final_yaw_err = wrap(target_yaw - self.yaw)
                    if abs(final_yaw_err) > YAW_TOL:
                        ang_out = self.ang_pid.compute(final_yaw_err, now)
                        cmd = Twist()
                        cmd.angular.z = ang_out
                        self.cmd_pub.publish(cmd)
                        continue

                print(f"  ✓ Reached '{label}' — dist_err={dist:.3f}m")
                self._wp_reached_times.append((now, label))
                self._stop()
                time.sleep(0.3)
                return

            if dist > HEADING_FREEZE_DIST:
                heading_err = wrap(math.atan2(dy, dx) - self.yaw)
                frozen_heading_err = heading_err
            else:
                heading_err = frozen_heading_err if frozen_heading_err is not None else 0.0

            if target_yaw is not None:
                final_yaw_err = wrap(target_yaw - self.yaw)
                alpha = math.exp(-dist / BLEND_RADIUS)
                ang_err = (1.0 - alpha) * heading_err + alpha * final_yaw_err
            else:
                ang_err = heading_err

            blend = math.cos(heading_err) ** 2
            ang_out = self.ang_pid.compute(ang_err, now)
            lin_raw = self.lin_pid.compute(dist, now)
            lin_out = max(0.0, lin_raw * blend)

            # Anti-orbit / anti-oscillation tuning near the goal.
            if dist < 0.50 and abs(heading_err) > math.radians(20):
                lin_out = 0.0
            if dist < 0.35:
                lin_out = min(lin_out, 0.06)
            if dist < 0.30:
                lin_out = 0.0

            cmd = Twist()
            cmd.linear.x = lin_out
            cmd.angular.z = ang_out
            self.cmd_pub.publish(cmd)

    def _run_waypoint_action(self, action):
        """Run any arm action associated with a waypoint."""
        if action == 'PICK_UP_STOW':
            print('Executing PICK -> UP -> STOW sequence...')
            self._send_arm(ARM_PICK)
            time.sleep(2.0)
            self._send_arm(ARM_UP)
            time.sleep(2.0)
            self._send_arm(ARM_STOW)
            time.sleep(2.0)
        elif action == 'DROP':
            print('Executing DROP sequence...')
            self._send_arm(ARM_STOW)
            time.sleep(2.0)

    def _print_summary(self, elapsed):
        print("\n" + "═" * 60)
        print('  NAVIGATION COMPLETE — SUMMARY')
        print("═" * 60)
        print(f'  Total run time     : {elapsed:.1f} s')
        print(f'  Total distance     : {self._total_distance:.2f} m')
        print(f"  Rooms visited      : {', '.join(sorted(self._rooms_visited))}")
        print(f'  Waypoints reached  : {len(self._wp_reached_times)} / {len(WAYPOINTS)}')
        print('\n  Waypoint log:')
        for timestamp, label in self._wp_reached_times:
            print(f'    [{timestamp:6.1f}s]  {label}')
        print("═" * 60)

    def run(self):
        print('Waiting for odometry...')
        while rclpy.ok() and not self.odom_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f'Odometry ready. Start pos=({self.x:.2f}, {self.y:.2f})\n')

        print('Stowing arm...')
        self._send_arm(ARM_STOW)
        time.sleep(2.0)
        print('Arm stowed. Beginning navigation.\n')

        print('─' * 60)
        print(
            f"  {'TIME':>7}  {'X':>7}  {'Y':>7}  {'YAW':>7}  "
            f"{'DIST_WP':>8}  {'TOTAL_M':>8}  ROOMS"
        )
        print('─' * 60)

        t0 = time.monotonic()

        for wp in WAYPOINTS:
            if not rclpy.ok():
                break

            if len(wp) == 5:
                tx, ty, label, yaw_deg, action = wp
                target_yaw = math.radians(yaw_deg)
            elif len(wp) == 4:
                tx, ty, label, action = wp
                target_yaw = None
            else:
                tx, ty, label = wp
                target_yaw = None
                action = None

            self._go_to(tx, ty, label, t0, target_yaw=target_yaw)
            self._run_waypoint_action(action)

        elapsed = time.monotonic() - t0
        self._print_summary(elapsed)


def main():
    rclpy.init()
    node = Navigator()
    try:
        node.run()
    except KeyboardInterrupt:
        print('\nNavigation interrupted by user.')
    finally:
        try:
            if rclpy.ok():
                node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
