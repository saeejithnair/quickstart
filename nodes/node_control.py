"""Main control node."""

import logging
import time

import numpy as np

import lib.constants as CFG
from lib.control.motor_control import MotorControl
from lib.planning.d_star import DStar
from lib.planning.lookahead_controller import LookaheadController
from lib.logging_utils import Logger
from lib.messages.localization_initialized_msg import LOCALIZATION_INITIALIZED_MSG
from lib.messages.robot_pose_msg import ROBOT_POSE_MSG
from lib.messages.target_velocity_msg import TARGET_VELOCITY_MSG
from lib.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from lib.messages.path_plan_msg import PATH_PLAN_MSG
from lib.messages.mqtt_utils import MQTTPublisher, MQTTSubscriber
from lib.messages.topic_to_message_type import (
    TOPIC_LOCALIZATION_INITIALIZED,
    TOPIC_ROBOT_POSE,
    TOPIC_TARGET_VELOCITY,
    TOPIC_WHEEL_VELOCITIES,
    TOPIC_PATH_PLAN
)

# Set logging level for matplotlib to WARNING
logging.getLogger('matplotlib').setLevel(logging.WARNING)


class ControlNode(object):
    """Control node class."""

    def __init__(self, logging_level = logging.INFO):
        """
        Initialize the ControlNode class.

        :param logging_level: The logging level to use for the logger.
        """
        self.logger = Logger('control', 'logs/control.log', level=logging_level)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_LOCALIZATION_INITIALIZED: LOCALIZATION_INITIALIZED_MSG,
                                                                                                TOPIC_TARGET_VELOCITY: TARGET_VELOCITY_MSG,
                                                                                                TOPIC_ROBOT_POSE: ROBOT_POSE_MSG,
                                                                                                TOPIC_PATH_PLAN: PATH_PLAN_MSG})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG})

        # Initialize message variables
        self.traversability_grid = None
        self.robot_pose_msg = None
        self.current_target_point_msg = None
        self.target_point_msg = None
        self.path_plan_msg = None

        self.dstar = DStar()  # Initialize D* pathfinding algorithm

        # Extract grid parameters from configuration
        self.grid_cell_size_m = CFG.MAPPING_GRID_GRID_CELL_SIZE
        self.grid_width_m = CFG.MAPPING_GRID_DEFAULT_PLANAR_SPREAD

        self.loop_rate_hz = 300

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        controller = MotorControl(logger=self.logger)

        self.localization_initialized = False
        self.target_velocity_msg = None
        self.previous_target_velocity_msg = None
        self.stopping_velocity_profile_linear = None
        self.stopping_velocity_profile_angular = None
        self.path_plan_msg = None

        # Initialize lookahead controller
        lookahead_controller = LookaheadController(lookahead_distance=0.25,
                                                   max_linear_velocity=0.3,
                                                   max_angular_velocity=1.5
                                                   )
        try:
            while True:
                start_time = time.time()

                localization_initialized_msg: LOCALIZATION_INITIALIZED_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_LOCALIZATION_INITIALIZED)
                target_velocity_msg: TARGET_VELOCITY_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_TARGET_VELOCITY)
                
                # Update path plan message
                path_plan_msg: PATH_PLAN_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_PATH_PLAN)
                if path_plan_msg is not None:
                    self.path_plan_msg = path_plan_msg

                # Check if path plan message is too old
                if self.path_plan_msg is not None and time.time() - self.path_plan_msg.timestamp > 5.0:
                    self.path_plan_msg = None

                # Get robot pose
                self.robot_pose_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE)                    

                if self.path_plan_msg is not None and self.robot_pose_msg is not None:
                    world_x_pos = self.robot_pose_msg.x_m
                    world_y_pos = self.robot_pose_msg.y_m
                    world_yaw = self.robot_pose_msg.theta_rad
                    # Trajectory planning
                    linear_velocity, angular_velocity = lookahead_controller.vel_request(self.path_plan_msg.path_pose_list, (world_x_pos, world_y_pos, world_yaw))
                    target_velocity_msg: TARGET_VELOCITY_MSG = TARGET_VELOCITY_MSG()
                    target_velocity_msg.timestamp = time.time()
                    target_velocity_msg.linear_velocity_mps = linear_velocity
                    target_velocity_msg.angular_velocity_radps = angular_velocity
                    self.previous_target_velocity_msg = target_velocity_msg
                elif target_velocity_msg is not None:
                    # Use the target velocity received on the topic if available
                    self.previous_target_velocity_msg = target_velocity_msg
                    self.stopping_velocity_profile_linear = None
                    self.stopping_velocity_profile_angular = None
                else:
                    # If no target velocity is received and no path plan is available, set the target velocity to 0
                    if self.previous_target_velocity_msg is not None and self.previous_target_velocity_msg.timestamp > time.time() - 2.0:
                        if self.stopping_velocity_profile_linear is None or self.stopping_velocity_profile_angular is None:
                            self.stopping_velocity_profile_linear, self.stopping_velocity_profile_angular = generate_stopping_velocity_profile(self.previous_target_velocity_msg, 2.0)
                        # Get the nearest timestamp match in the profile
                        nearest_timestamp_linear = min(self.stopping_velocity_profile_linear.keys(), key=lambda x: abs(x - time.time()))
                        nearest_timestamp_angular = min(self.stopping_velocity_profile_angular.keys(), key=lambda x: abs(x - time.time()))
                        linear_velocity_mps = self.stopping_velocity_profile_linear[nearest_timestamp_linear]
                        angular_velocity_radps = self.stopping_velocity_profile_angular[nearest_timestamp_angular]
                        target_velocity_msg: TARGET_VELOCITY_MSG = TARGET_VELOCITY_MSG()
                        target_velocity_msg.linear_velocity_mps = linear_velocity_mps
                        target_velocity_msg.angular_velocity_radps = angular_velocity_radps
                        target_velocity_msg.timestamp = time.time()
                    else:
                        self.previous_target_velocity_msg = None
                        self.stopping_velocity_profile = None
                        target_velocity_msg: TARGET_VELOCITY_MSG = TARGET_VELOCITY_MSG()
                        target_velocity_msg.timestamp = time.time()
                        target_velocity_msg.linear_velocity_mps = 0.0
                        target_velocity_msg.angular_velocity_radps = 0.0

                if localization_initialized_msg is not None:
                    self.localization_initialized = localization_initialized_msg.initialized

                if self.localization_initialized:
                    controller.set_linear_angular_velocities(target_velocity_msg.linear_velocity_mps, target_velocity_msg.angular_velocity_radps)

                    l_vel_mps = controller.get_left_motor_velocity()
                    r_vel_mps = controller.get_right_motor_velocity()
                    wheel_velocities_msg = WHEEL_VELOCITIES_DATA_MSG()
                    wheel_velocities_msg.timestamp = time.time()
                    wheel_velocities_msg.left_vel_mps = l_vel_mps
                    wheel_velocities_msg.right_vel_mps = r_vel_mps
                    self.mqtt_publisher.publish_msg(TOPIC_WHEEL_VELOCITIES, wheel_velocities_msg)

                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("Control node stopped by user.")
        finally:
            controller.stop()
            self.mqtt_subscriber.stop()
            self.mqtt_publisher.stop()

def generate_stopping_velocity_profile(target_velocity_msg: TARGET_VELOCITY_MSG, decel_time_s: float):
    # Generate a velocity profile that decelerates to 0 in decel_time_s seconds
    linear_velocity_mps = target_velocity_msg.linear_velocity_mps
    angular_velocity_radps = target_velocity_msg.angular_velocity_radps
    time_to_stop = decel_time_s
    resolution = 0.01  # 10 ms resolution
    time_steps = np.arange(0, time_to_stop + resolution, resolution)
    deceleration_rate_linear = linear_velocity_mps / time_to_stop
    deceleration_rate_angular = angular_velocity_radps / time_to_stop
    curr_time = time.time()
    stopping_velocity_profile_linear = {t + curr_time: linear_velocity_mps - deceleration_rate_linear * t for t in time_steps}
    stopping_velocity_profile_angular = {t + curr_time: angular_velocity_radps - deceleration_rate_angular * t for t in time_steps}
    return stopping_velocity_profile_linear, stopping_velocity_profile_angular

if __name__ == "__main__":
    control_node = ControlNode()
    control_node.run()
