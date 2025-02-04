import time

import numpy as np

from pymlg.numpy import SO2

from lib.planning.pid import PID_ctrl, PID_type


class LookaheadController:
    """
    This class is a simple lookahead controller that computes a trajectory to follow a path given by a list of poses.
    """

    def __init__(self, lookahead_distance, max_linear_velocity, max_angular_velocity,
                 pid_type:PID_type = PID_type.PID,
                 pid_linear_kp=0.3, pid_linear_kv=0.0, pid_linear_ki=0.0,
                 pid_angular_kp=0.3, pid_angular_kv=0.0, pid_angular_ki=0.0):
        """
        Initialize the LookaheadController with given parameters.

        :param lookahead_distance: The distance to look ahead on the path.
        :param max_linear_velocity: The maximum linear velocity.
        :param max_angular_velocity: The maximum angular velocity.
        :param pid_type: The type of PID controller.
        :param pid_linear_kp: The proportional gain for linear velocity.
        :param pid_linear_kv: The derivative gain for linear velocity.
        :param pid_linear_ki: The integral gain for linear velocity.
        :param pid_angular_kp: The proportional gain for angular velocity.
        :param pid_angular_kv: The derivative gain for angular velocity.
        :param pid_angular_ki: The integral gain for angular velocity.
        """
        self.lookahead_distance = lookahead_distance
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self.pid_linear = PID_ctrl(pid_type, pid_linear_kp, pid_linear_kv, pid_linear_ki)
        self.pid_angular = PID_ctrl(pid_type, pid_angular_kp, pid_angular_kv, pid_angular_ki)

        # TODO: trying this:
        self.goal_idx = 0

    @staticmethod
    def calculate_linear_error(current_pose, goal_pose):
        """
        Calculate the linear error between the current pose and the goal pose.

        :param current_pose: The current pose of the robot.
        :param goal_pose: The goal pose to reach.
        :return: The linear error.
        """
        return np.sqrt((current_pose[0] - goal_pose[0])**2 +
                       (current_pose[1] - goal_pose[1])**2)

    @staticmethod
    def calculate_angular_error(current_pose, goal_pose):
        """
        Calculate the angular error between the current pose and the goal pose.

        :param current_pose: The current pose of the robot.
        :param goal_pose: The goal pose to reach.
        :return: The angular error.
        """
        # error_angular = -np.arctan2(goal_pose[1] - current_pose[1],
        #                            goal_pose[0] - current_pose[0]) + np.pi/2 - current_pose[2]
        
        # # Normalize the angular error to be within [-pi, pi]
        # if error_angular <= -np.pi:
        #     error_angular += 2 * np.pi
        # elif error_angular >= np.pi:
        #     error_angular -= 2 * np.pi

        # reproject to lie group then to rotation vector equivalent to get error

        # generate C_k \in SO_(2), representing current pose
        C_k = SO2.Exp(current_pose[2])

        # generate C_target \in SO_(2), representing target pose

        # generate unit vector pointing from current pose to target pose
        u_k = np.array([goal_pose[0] - current_pose[0], goal_pose[1] - current_pose[1]], dtype=np.float64)
        u_k /= np.linalg.norm(u_k)
        theta = np.arccos(np.dot(np.array([0, 1]), u_k))

        # pick a sense for the rotation based on the greater dot product between the unit vector vs. [1, 0], [-1, 0]

        if np.dot(np.array([1, 0]), u_k) > np.dot(np.array([-1, 0]), u_k):
            theta = -theta

        C_target = SO2.Exp(theta)

        # generate error rotation matrix and corresponding theta
        C_error = np.dot(C_target, C_k.T)
        theta_error = SO2.Log(C_error)
        
        return theta_error
    
    def vel_request(self, path_pose_list, current_pose):

        goal = self.find_goal_pose(path_pose_list, current_pose)

        finalGoal = path_pose_list[-1]

        error_linear = LookaheadController.calculate_linear_error(current_pose, finalGoal)
        error_angular = LookaheadController.calculate_angular_error(current_pose, goal)

        print(f"Goal: {goal}, Current: {current_pose}")
        print(f"Linear Err: {error_linear}, Angular Err: {error_angular}")

        if abs(error_angular) > np.pi/2:
            linear_velocity = 0
        else:
            linear_velocity = self.pid_linear.update(error_linear, time.time(), True)

        angular_velocity = self.pid_angular.update(error_angular, time.time(), True)

        linear_velocity = np.clip(linear_velocity, -self.max_linear_velocity, self.max_linear_velocity)
        angular_velocity = np.clip(angular_velocity, -self.max_angular_velocity, self.max_angular_velocity)

        return linear_velocity, angular_velocity

    def find_goal_pose(self, path_pose_list, current_pose):
        """
        Find the goal pose within the lookahead distance.

        :param path_pose_list: List of poses representing the path.
        :param current_pose: The current pose of the robot.
        :return: The goal pose within the lookahead distance.
        """
        current_x, current_y, current_yaw = current_pose

        # ensure contiguous tracking by making sure we don't go back to old targets
        for idx, pose in enumerate(path_pose_list[self.goal_idx:]):
            distance = np.sqrt((pose[0] - current_x) ** 2 + (pose[1] - current_y) ** 2)
            if distance >= self.lookahead_distance:
                self.goal_idx = idx
                return pose
        return path_pose_list[-1]  # Return the last pose if no pose is found within the lookahead distance
