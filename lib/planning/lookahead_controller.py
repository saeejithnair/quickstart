import time

import numpy as np

from lib.planning.pid import PID_ctrl, PID_type


class LookaheadController:
    """
    This class is a simple lookahead controller that computes a trajectory to follow a path given by a list of poses.
    """

    def __init__(self, lookahead_distance, max_linear_velocity, max_angular_velocity,
                 pid_type:PID_type = PID_type.PID,
                 pid_linear_kp=0.4, pid_linear_kv=0.0, pid_linear_ki=0.0,
                 pid_angular_kp=1.2, pid_angular_kv=0.0, pid_angular_ki=0.0):
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

        # Construct rotation matrices for current and goal poses
        R_current = np.array([
            [np.cos(current_pose[2]), -np.sin(current_pose[2]), 0],
            [np.sin(current_pose[2]), np.cos(current_pose[2]), 0],
            [0, 0, 1]
        ])

        goal_theta = np.arctan2(goal_pose[0] - current_pose[0], goal_pose[1] - current_pose[1])
        goal_pose = [goal_pose[0], goal_pose[1], goal_theta]
        R_goal = np.array([
            [np.cos(goal_pose[2]), -np.sin(goal_pose[2]), 0],
            [np.sin(goal_pose[2]), np.cos(goal_pose[2]), 0],
            [0, 0, 1]
        ])

        # Compute the relative rotation matrix
        R_relative = np.dot(R_goal.T, R_current)

        # Extract the angular error from the relative rotation matrix
        error_angular = np.arctan2(R_relative[1, 0], R_relative[0, 0])

        return error_angular
    
    def vel_request(self, path_pose_list, current_pose):

        goal = self.find_goal_pose(path_pose_list, current_pose)

        finalGoal = path_pose_list[-1]

        error_linear = LookaheadController.calculate_linear_error(current_pose, finalGoal)
        error_angular = LookaheadController.calculate_angular_error(current_pose, goal)

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
        poseArray=np.array([current_pose[0], current_pose[1]]) 
        listGoalsArray=np.array(path_pose_list)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return path_pose_list[ min(closestIndex + 1, len(path_pose_list) - 1) ]
