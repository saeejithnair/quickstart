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
from lib.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from lib.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from lib.messages.robot_pose_msg import ROBOT_POSE_MSG
from lib.messages.target_point_msg import TARGET_POINT_MSG
from lib.messages.target_velocity_msg import TARGET_VELOCITY_MSG
from lib.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from lib.messages.mqtt_utils import MQTTPublisher, MQTTSubscriber
from lib.messages.topic_to_message_type import (
    TOPIC_LOCALIZATION_INITIALIZED,
    TOPIC_ROBOT_POSE,
    TOPIC_ROBOT_POSE_GRID_COORDS,
    TOPIC_TARGET_POINT,
    TOPIC_TARGET_VELOCITY,
    TOPIC_TRAVERSABILITY_GRID,
    TOPIC_WHEEL_VELOCITIES,
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

        # Extract grid parameters from configuration
        self.grid_cell_size_m = CFG.MAPPING_GRID_GRID_CELL_SIZE
        self.grid_width_m = CFG.MAPPING_GRID_DEFAULT_PLANAR_SPREAD

        self.loop_rate_hz = 300

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        controller = MotorControl(logger=self.logger)

        self.velocity_target = 0.0
        self.yaw_rate_target = 0.0
        self.localization_initialized = False
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

                localization_initialized_msg: LOCALIZATION_INITIALIZED_MSG = self.mqtt_subscriber.get_latest_message(TOPIC_LOCALIZATION_INITIALIZED)
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

                            print("Target point has been reached, not replanning.")
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
                        print("Path found:", self.path_pose_list[0], self.path_pose_list[-1])
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
                    print(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")
                elif target_velocity_msg is not None:
                    # use the velocity received on the topic
                    pass
                # else:
                #     target_velocity_msg: TARGET_VELOCITY_MSG = TARGET_VELOCITY_MSG()
                #     target_velocity_msg.linear_velocity_mps = 0.0
                #     target_velocity_msg.angular_velocity_radps = 0.0

                if localization_initialized_msg is not None:
                    self.localization_initialized = localization_initialized_msg.initialized

                if target_velocity_msg is not None:
                    print("Target velocity recieved, setting targets.")
                    print("Linear velocity target: ", target_velocity_msg.linear_velocity_mps)
                    print("Angular velocity target: ", target_velocity_msg.angular_velocity_radps)
                    self.velocity_target = target_velocity_msg.linear_velocity_mps
                    self.yaw_rate_target = target_velocity_msg.angular_velocity_radps

                # Call the balance_step function with the updated imu_data
                if self.localization_initialized:
                    controller.set_linear_angular_velocities(self.velocity_target, self.yaw_rate_target)

                    l_vel_mps = controller.get_left_motor_velocity()
                    r_vel_mps = controller.get_right_motor_velocity()
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
            self.logger.info("Control node stopped by user.")
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
    control_node = ControlNode()
    control_node.run()
