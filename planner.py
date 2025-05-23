import math
import numpy as np

class Planner:
    def __init__(self, waypoints=None, path=None, waypoint_tolerance=0.2):
        """
        Initialize the Planner.
        """
        self.waypoints = waypoints if waypoints else {}
        self.path = path if path else []
        self.waypoint_tolerance = waypoint_tolerance
        self.current_index = 0  # Index of the current waypoint in the path

        # PD controller variables for linear velocity
        self.linear_error_prev = 0.0

        # PD controller variables for angular velocity
        self.angular_error_prev = 0.0

        # PD gains
        self.Kp_v = 3.0  # Proportional gain for linear velocity
        self.Kd_v = 0.1  # Derivative gain for linear velocity

        self.Kp_w = 0.2  # Proportional gain for angular velocity
        self.Kd_w = 0.01  # Derivative gain for angular velocity

    @staticmethod
    def angle_diff(theta1, theta2):
        """Normalize angles in radians"""
        diff = theta2 - theta1
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def get_current_target(self):
        """
        Return the current target waypoint and its position.
        """
        if self.current_index >= len(self.path):
            return None, None  # No more waypoints to follow
        waypoint_name = self.path[self.current_index]
        target_position = self.waypoints[waypoint_name]
        return waypoint_name, target_position
    

    def update(self, current_pose):
        """
        Update the planner with current robot pose. 
        """
        if self.current_index >= len(self.path):
            # No more waypoints to follow
            return 0.0, 0.0

        # Get the current target waypoint
        waypoint_name = self.path[self.current_index]
        target = self.waypoints[waypoint_name]

        # Compute distance to the target
        dist_to_target = math.hypot(current_pose[0] - target[0], current_pose[1] - target[1])

        if dist_to_target < self.waypoint_tolerance:
            # If the waypoint is reached, move to the next waypoint
            print(f"Reached {waypoint_name}")
            self.current_index += 1
            if self.current_index >= len(self.path):
                # If all waypoints are reached, stop
                return 0.0, 0.0
            waypoint_name = self.path[self.current_index]
            target = self.waypoints[waypoint_name]

        # Compute control commands to move toward the target
        x, y, theta = current_pose
        x_target, y_target = target
        dx = x_target - x
        dy = y_target - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.angle_diff(theta, target_angle + np.pi)

        # PD Controller for Linear Velocity
        linear_error = distance
        linear_error_derivative = linear_error - self.linear_error_prev
        self.linear_error_prev = linear_error

        v = (self.Kp_v * linear_error +
             self.Kd_v * linear_error_derivative)

        # PD Controller for Angular Velocity
        angular_error = angle_error
        angular_error_derivative = angular_error - self.angular_error_prev
        self.angular_error_prev = angular_error

        w = (self.Kp_w * angular_error +
             self.Kd_w * angular_error_derivative)

        # Limit speeds
        v = max(min(v, 1.0), -1.0)
        w = max(min(w, 1.0), -1.0)

        return v, w