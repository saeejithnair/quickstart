import logging
import os
import time

import numpy as np
import yaml

import utils.constants as CFG
from nodes.control.lqr_balance import BalanceController
from nodes.control.motor_control import MotorControl
from nodes.planning.d_star import DStar
from nodes.planning.lookahead_controller import LookaheadController
from utils.logging_utils import Logger
from utils.messages.gyro_data_msg import GYRO_DATA_MSG
from utils.messages.localization_initialized_msg import LOCALIZATION_INITIALIZED_MSG
from utils.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from utils.messages.robot_extended_pose_msg import ROBOT_EXTENDED_POSE_MSG
from utils.messages.target_velocity_msg import TARGET_VELOCITY_MSG
from utils.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from utils.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from utils.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from utils.messages.robot_pose_msg import ROBOT_POSE_MSG
from utils.messages.target_point_msg import TARGET_POINT_MSG

from utils.mqtt_utils import MQTTPublisher, MQTTSubscriber
from utils.topic_to_message_type import (
    TOPIC_GYRO,
    TOPIC_LOCALIZATION_INITIALIZED,
    TOPIC_PROCESSED_IMU,
    TOPIC_RAW_IMU,
    TOPIC_ROBOT_EXTENDED_POSE,
    TOPIC_TARGET_VELOCITY,
    TOPIC_WHEEL_VELOCITIES,
    TOPIC_ROBOT_POSE,
    TOPIC_ROBOT_POSE_GRID_COORDS,
    TOPIC_TARGET_POINT,
    TOPIC_TRAVERSABILITY_GRID
)

# Set logging level for matplotlib to WARNING
logging.getLogger('matplotlib').setLevel(logging.WARNING)


class BalanceControl(object):
    def __init__(self, logging_level = logging.INFO):
        """
        Initialize the BalanceControl class.

        :param logging_level: The logging level to use for the logger.
        """
        self.logger = Logger('control', 'logs/control.log', level=logging_level)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_GYRO: GYRO_DATA_MSG,
                                                                                                TOPIC_RAW_IMU: RAW_IMU_DATA_MSG,
                                                                                                TOPIC_LOCALIZATION_INITIALIZED: LOCALIZATION_INITIALIZED_MSG,
                                                                                                TOPIC_ROBOT_EXTENDED_POSE: ROBOT_EXTENDED_POSE_MSG,
                                                                                                TOPIC_PROCESSED_IMU: RAW_IMU_DATA_MSG,
                                                                                                TOPIC_TARGET_VELOCITY: TARGET_VELOCITY_MSG,
                                                                                                TOPIC_TRAVERSABILITY_GRID: OCCUPANCY_GRID_MSG,
                                                                                                TOPIC_ROBOT_POSE: ROBOT_POSE_MSG,
                                                                                                TOPIC_ROBOT_POSE_GRID_COORDS: ROBOT_POSE_GRID_COORDS_MSG,
                                                                                                TOPIC_TARGET_POINT: TARGET_POINT_MSG})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG})

        # Initialize message variables
        self.traversability_grid = None
        self.robot_pose_msg = None
        self.robot_pose_grid_coords_msg = None
        self.current_target_point_msg = None
        self.target_point_msg = None

        self.dstar = DStar()  # Initialize D* pathfinding algorithm

        # Grid parameters
        # Retrieve yaml file location
        map_config_loc = os.getenv("BASE_CONFIG_LOC", CFG.MAPPING_BASE_CONFIG_LOC)
        # Add /home/$USER to the beginning of the path
        map_config_loc = f"/home/{os.getenv('USER')}/{map_config_loc}"

        # Retrieve configuration yaml
        with open(map_config_loc, 'r') as stream:
            self.map_config = yaml.safe_load(stream)

        # Extract grid parameters from configuration
        self.grid_cell_size_m = self.map_config['grid_params']['grid_cell_size']
        self.grid_width_m = self.map_config['grid_params']['default_planar_spread']


        self.loop_rate_hz = 300

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        enable_motor_control = os.getenv("ENABLE_MOTOR_CONTROL", CFG.MOTOR_CONTROL_ENABLE_MOTOR_CONTROL)
        # controller = BalanceController(standalone=False,
        #                                 enable_motor_control=True if enable_motor_control == "1" else False,
        #                                 logger=self.logger)
        controller = MotorControl(enable_motor_control=True if enable_motor_control == "1" else False,
                                  logger=self.logger)

        # Initialize variables to store sensor data
        imu_data = None
        raw_imu_data_dict = None

        self.velocity_target = 0.0
        self.yaw_rate_target = 0.0
        self.localization_initialized = False
        self.robot_extended_pose_msg = None
        self.processed_imu_msg = None
        self.target_velocity_msg = None

        replan_time = time.time()  # Initialize replan time
        # Initialize lookahead controller
        lookahead_controller = LookaheadController(lookahead_distance=0.25,
                                                   max_linear_velocity=0.5,
                                                   max_angular_velocity=1.0
                                                   )
        self.prev_goal_pose = None
        self.path_pose_list = None

        try:
            while True:
                start_time = time.time()

                gyro_data = self.mqtt_subscriber.get_latest_message(TOPIC_GYRO)
                raw_imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_RAW_IMU)
                localization_initialized_msg: LOCALIZATION_INITIALIZED_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_LOCALIZATION_INITIALIZED)
                robot_extended_pose_msg: ROBOT_EXTENDED_POSE_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_EXTENDED_POSE)
                processed_imu_msg: RAW_IMU_DATA_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_PROCESSED_IMU)
                target_velocity_msg: TARGET_VELOCITY_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_TARGET_VELOCITY)

                # Get traversability grid
                traversability_grid = self.mqtt_subscriber.get_latest_message(TOPIC_TRAVERSABILITY_GRID)
                if traversability_grid is not None:
                    self.traversability_grid = traversability_grid

                # Get robot pose
                robot_pose_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE)
                if robot_pose_msg is not None:
                    self.robot_pose_msg = robot_pose_msg

                # Get robot pose in grid coordinates
                robot_pose_grid_coords_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE_GRID_COORDS)
                if robot_pose_grid_coords_msg is not None:
                    self.robot_pose_grid_coords_msg = robot_pose_grid_coords_msg

                if self.robot_pose_grid_coords_msg is not None:
                    grid_start_pose_x = self.robot_pose_grid_coords_msg.x_grid
                    grid_start_pose_y = self.robot_pose_grid_coords_msg.y_grid
                else:
                    grid_start_pose_x = None
                    grid_start_pose_y = None

                # Get target point message
                target_point_msg = self.mqtt_subscriber.get_latest_message(TOPIC_TARGET_POINT)
                if target_point_msg is not None:
                    self.target_point_msg = target_point_msg
                    if self.current_target_point_msg is None:
                        self.current_target_point_msg = target_point_msg

                if self.target_point_msg is not None:
                    try:
                        grid_goal_pose_x = self.target_point_msg.x_grid
                        grid_goal_pose_y = self.target_point_msg.y_grid
                    except:
                        print("Target point is not in the correct format", self.target_point_msg)
                        continue

                    # If within tolerance of the target point, don't replan
                    if grid_start_pose_x is not None and grid_start_pose_y is not None:
                        if np.linalg.norm([grid_start_pose_x - grid_goal_pose_x, grid_start_pose_y - grid_goal_pose_y]) < 3:
                            self.target_point_msg = None
                            self.path_pose_list = None
                            self.traversability_grid = None
                            continue

                # Process traversability grid
                if self.traversability_grid is not None and self.robot_pose_msg is not None:
                    traversability_grid = process_traversability_grid(self.traversability_grid)
                    world_x_pos = self.robot_pose_msg.x_m
                    world_y_pos = self.robot_pose_msg.y_m
                    world_yaw = self.robot_pose_msg.theta_rad

                # Check if it's time to replan
                if time.time() - replan_time > 5.0 and self.target_point_msg is not None and self.traversability_grid is not None and self.robot_pose_grid_coords_msg is not None and self.robot_pose_msg is not None:

                    print(f"Attempting to plan from ({grid_start_pose_x}, {grid_start_pose_y}) to ({grid_goal_pose_x}, {grid_goal_pose_y})")

                    # Initialize D* algorithm and run pathfinding
                    self.dstar.initialize(traversability_grid, (grid_start_pose_y, grid_start_pose_x), (grid_goal_pose_y, grid_goal_pose_x))
                    path = self.dstar.run()

                    if path:
                        # Convert path to world coordinates
                        self.path_pose_list = [convert_grid_coords_to_pose(path[i], self.grid_cell_size_m, self.grid_width_m) for i in range(len(path))]
                        # Print every other point in the path
                        print(self.path_pose_list[::2])
                        # print("Path found:", self.path_pose_list[0], self.path_pose_list[-1])
                        self.dstar.plot_path()
                    else:
                        self.path_pose_list = None
                        self.dstar.plot_path()
                        print("No path found")
                        continue

                    replan_time = time.time()

                if self.path_pose_list is not None:
                    # Trajectory planning
                    linear_velocity, angular_velocity = lookahead_controller.vel_request(self.path_pose_list, (world_x_pos, world_y_pos, world_yaw))
                    target_velocity_msg: TARGET_VELOCITY_MSG = TARGET_VELOCITY_MSG()
                    target_velocity_msg.linear_velocity_mps = linear_velocity
                    target_velocity_msg.angular_velocity_radps = angular_velocity
                elif target_velocity_msg is not None:
                    # use the velocity received on the topic
                    pass
                else:
                    target_velocity_msg: TARGET_VELOCITY_MSG = TARGET_VELOCITY_MSG()
                    target_velocity_msg.linear_velocity_mps = 0.0
                    target_velocity_msg.angular_velocity_radps = 0.0

                if localization_initialized_msg is not None:
                    self.localization_initialized = localization_initialized_msg.initialized

                if robot_extended_pose_msg is not None:
                    self.robot_extended_pose_msg = robot_extended_pose_msg

                if processed_imu_msg is not None:
                    self.processed_imu_msg = processed_imu_msg

                if gyro_data is not None:
                    timestamp = gyro_data.timestamp
                    imu_data = {}
                    imu_data['orientation'] = [0, 0, 0]
                    imu_data['orientation'][0] = gyro_data.pitch
                    imu_data['orientation'][1] = gyro_data.roll
                    imu_data['orientation'][2] = gyro_data.yaw
                else:
                    imu_data = None

                if raw_imu_data is not None:
                    raw_imu_data_dict = {}
                    raw_imu_data_dict['accel'] = [0, 0, 0]
                    raw_imu_data_dict['accel'][0] = raw_imu_data.accel_x
                    raw_imu_data_dict['accel'][1] = raw_imu_data.accel_y
                    raw_imu_data_dict['accel'][2] = raw_imu_data.accel_z
                    raw_imu_data_dict['gyro'] = [0, 0, 0]
                    raw_imu_data_dict['gyro'][0] = raw_imu_data.gyro_x
                    raw_imu_data_dict['gyro'][1] = raw_imu_data.gyro_y
                    raw_imu_data_dict['gyro'][2] = raw_imu_data.gyro_z
                else:
                    raw_imu_data_dict = None

                if target_velocity_msg is not None:
                    self.velocity_target = target_velocity_msg.linear_velocity_mps
                    self.yaw_rate_target = target_velocity_msg.angular_velocity_radps

                # Call the balance_step function with the updated imu_data
                if imu_data is not None and raw_imu_data_dict is not None and self.localization_initialized:
                    l_vel_mps, r_vel_mps = controller.control_step(gyro_filtered=imu_data['orientation'],
                                                                    imu_gyro_data=raw_imu_data_dict['gyro'],
                                                                    velocity_target_mps=self.velocity_target,
                                                                    yaw_rate_target_rad_s=self.yaw_rate_target,
                                                                    robot_pose=self.robot_extended_pose_msg)

                    wheel_velocities_msg = WHEEL_VELOCITIES_DATA_MSG()
                    wheel_velocities_msg.timestamp = time.time()
                    wheel_velocities_msg.left_vel_mps = l_vel_mps
                    wheel_velocities_msg.right_vel_mps = r_vel_mps
                    self.mqtt_publisher.publish_msg(TOPIC_WHEEL_VELOCITIES, wheel_velocities_msg)

                # Reset variables
                self.robot_pose_msg = None
                self.robot_pose_grid_coords_msg = None

                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("Balance stopped by user.")
        finally:
            controller.stop()
            self.mqtt_subscriber.stop()
            self.mqtt_publisher.stop()

def convert_grid_coords_to_pose(grid_coords, grid_cell_size_m: float, grid_width_m: float):
    """
    Convert grid coordinates to world coordinates.

    Parameters
    ----------
    grid_coords : tuple
        The grid coordinates (x, y).
    grid_cell_size_m : float
        The size of each grid cell in meters.
    grid_width_m : float
        The width of the grid in meters.

    Returns
    -------
    tuple
        (x, y) in world coordinates.
    """
    offset = grid_width_m / 2
    return (-(grid_coords[1] * grid_cell_size_m - offset), grid_coords[0] * grid_cell_size_m - offset)

def process_traversability_grid(traversability_grid: OCCUPANCY_GRID_MSG):
    """
    Process the traversability grid message.

    Parameters
    ----------
    traversability_grid : OCCUPANCY_GRID_MSG
        The traversability grid message.

    Returns
    -------
    np.array
        2D numpy array of the traversability grid.
    """
    width = traversability_grid.width

    # Convert the flattened list back to a 2D numpy array
    grid_array = np.array(traversability_grid.flattened_grid_list).reshape((width, width))

    return grid_array

if __name__ == "__main__":
    balance_control = BalanceControl()
    balance_control.run()
