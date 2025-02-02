import logging
import os
import time
import math

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import matplotlib
import lib.constants as CFG
from lib.logging_utils import Logger
from lib.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from lib.messages.gyro_data_msg import GYRO_DATA_MSG
from lib.messages.mqtt_message_base import MqttMessageBase
from lib.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from lib.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from lib.messages.tof_map_msg import TOF_MAP_MSG
from lib.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG
from lib.messages.wavemap_occupied_points_msg import WAVEMAP_OCCUPIED_POINTS_MSG
from lib.messages.mqtt_utils import MQTTSubscriber
from lib.messages.topic_to_message_type import (
    TOPIC_EXTENDED_POSE_W_BIAS,
    TOPIC_GYRO,
    TOPIC_OCCUPANCY_GRID,
    TOPIC_PROCESSED_IMU,
    TOPIC_RAW_IMU_UNSCALED_BIASED,
    TOPIC_WHEEL_VELOCITIES,
    TOPIC_TOF_MAP,
    TOPIC_WAVEMAP_OCCUPIED_POINTS,
)


class RerunViewer:
    def __init__(self, logging_level=logging.INFO):
        """
        Initialize the RerunViewer with logging and MQTT subscriber setup.

        :param logging_level: The logging level to use for the logger.
        """
        self.logger = Logger('rerun_viewer', 'logs/rerun_viewer.log', level=logging_level)
        self.ip_address = os.getenv("HOST_IP_ADDRESS", CFG.RERUN_HOST_IP_ADDRESS)

        self.last_periodic_log_time = time.time()
        self.last_delayed_log_time = time.time()

        # Color Mapping Setup
        self.cmap = matplotlib.colormaps["coolwarm"]  # Use a colormap that transitions from blue to red
        self.norm = matplotlib.colors.Normalize(vmin=0.0, vmax=4.0)  # 0–4 meters color range

        # Robot Box
        self.robot_half_size = np.array([[0.02, 0.02, 0.7]])   # half extents in x,y,z
        self.robot_color     = np.array([[1.0, 0.0, 0.0, 0.4]])# RGBA: red, semi-transparent

        # Robot Path
        # -----------------------------------------------------------------------------
        self.robot_path = []  # List to store robot positions over time
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}  # Robot's current pose

        # Initialize MQTT subscriber with topic to message type mapping
        self.mqtt_subscriber = MQTTSubscriber(
            broker_address="localhost",
            topic_to_message_map={
                TOPIC_GYRO: GYRO_DATA_MSG,
                TOPIC_OCCUPANCY_GRID: OCCUPANCY_GRID_MSG,
                TOPIC_RAW_IMU_UNSCALED_BIASED: RAW_IMU_DATA_MSG,
                TOPIC_WHEEL_VELOCITIES: WHEEL_VELOCITIES_DATA_MSG,
                TOPIC_EXTENDED_POSE_W_BIAS: EXTENDED_POSE_W_BIAS_MSG,
                TOPIC_PROCESSED_IMU: RAW_IMU_DATA_MSG,
                TOPIC_WAVEMAP_OCCUPIED_POINTS: WAVEMAP_OCCUPIED_POINTS_MSG,
            }
        )

        self.loop_rate_hz = 50

    def setup_rerun(self):
        """
        Setup the Rerun dashboard with the specified blueprint and connect to the server.
        """
        rr.init("robot_dashboard",spawn=False)
        rr.connect_tcp(f"{self.ip_address}:9876")

    def run(self):
        """
        Start the Rerun viewer, process input data, and handle graceful shutdown.
        """
        self.setup_rerun()
        self.mqtt_subscriber.start()
        try:
            while True:
                start_time = time.time()
                self.process_input()
                end_time = time.time()
                sleep_time = 1 / self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            self.logger.info("Rerun viewer stopped by user.")
            self.mqtt_subscriber.stop()

    def process_input(self, event=None):
        """
        Process incoming MQTT messages and log data periodically.

        :param event: Optional event parameter for future use.
        """
        # Retrieve latest messages from MQTT topics
        self.imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_GYRO)
        self.imu_us_ub_data = self.mqtt_subscriber.get_latest_message(TOPIC_RAW_IMU_UNSCALED_BIASED)
        self.wheel_ang_velocities = self.mqtt_subscriber.get_latest_message(TOPIC_WHEEL_VELOCITIES)
        self.extended_pose_data = self.mqtt_subscriber.get_latest_message(TOPIC_EXTENDED_POSE_W_BIAS)
        self.processed_imu_data = self.mqtt_subscriber.get_latest_message(TOPIC_PROCESSED_IMU)
        self.occupancy_grid_data = self.mqtt_subscriber.get_latest_message(TOPIC_OCCUPANCY_GRID)
        self.tof_map_data = self.mqtt_subscriber.get_latest_message(TOPIC_TOF_MAP)
        self.wavemap_occupied_points_data = self.mqtt_subscriber.get_latest_message(TOPIC_WAVEMAP_OCCUPIED_POINTS)

        current_time = time.time()
        # Log data every 0.1 seconds
        if current_time - self.last_periodic_log_time >= 0.1:
            if self.imu_data is not None:
                self.log_msg_values('gyro', self.imu_data)
            if self.imu_us_ub_data is not None:
                self.log_msg_values('raw_imu_unscaled_biased', self.imu_us_ub_data)
            if self.wheel_ang_velocities is not None:
                self.log_msg_values('wheel_velocities_mps', self.wheel_ang_velocities)
            if self.extended_pose_data is not None:
                self.log_msg_values('extended_pose_state', self.extended_pose_data)
            if self.processed_imu_data is not None:
                self.log_msg_values('processed_imu', self.processed_imu_data)
                rr.log("processed_imu", rr.TextLog(
                    f"Timestamp: {self.processed_imu_data.timestamp:.6f}\n"
                    f"Gyro X: {self.processed_imu_data.gyro_x:.2f}°/s\n"
                    f"Gyro Y: {self.processed_imu_data.gyro_y:.2f}°/s\n"
                    f"Gyro Z: {self.processed_imu_data.gyro_z:.2f}°/s\n"
                    f"Accel X: {self.processed_imu_data.accel_x:.2f}g\n"
                    f"Accel Y: {self.processed_imu_data.accel_y:.2f}g\n"
                    f"Accel Z: {self.processed_imu_data.accel_z:.2f}g"
                ))

            self.last_periodic_log_time = current_time

        if current_time - self.last_delayed_log_time >= 0.5:
            self.log_occupancy_grid(self.occupancy_grid_data)
            self.log_tof_map(self.tof_map_data)
            self.log_odometry(self.extended_pose_data)
            self.log_wavemap_occupied_points(self.wavemap_occupied_points_data)
            self.last_delayed_log_time = current_time

    def log_msg_values(self, id, MqttMessage: MqttMessageBase):
        """
        Log message values to Rerun, filtering out private and None attributes.

        :param id: Identifier for the message type.
        :param MqttMessage: The message object containing data to log.
        """
        for name, val in MqttMessage.__dict__.items():
            if not name.startswith('_') and val is not None:
                rr.log("{}/{}".format(id, name), rr.Scalar(val))

    def log_occupancy_grid(self, occupancy_grid_data: OCCUPANCY_GRID_MSG):
        """Custom handling of logging occupancy grid data to Rerun."""
        if occupancy_grid_data is not None:
            # Assuming you know the width and height of the grid
            width = occupancy_grid_data.width

            # Convert the flattened list back to a 2D numpy array
            grid_array = np.array(occupancy_grid_data.flattened_grid_list).reshape((width, width))

            occupied_indices = np.argwhere(grid_array == True)
            if occupied_indices.size > 0:
                # Convert grid indices to robot-local coordinates
                # Grid indices: row (y), col (x)
                local_x = occupied_indices[:, 1] * CFG.MAPPING_GRID_GRID_CELL_SIZE
                local_y = occupied_indices[:, 0] * CFG.MAPPING_GRID_GRID_CELL_SIZE
                local_z = np.full_like(local_x, 0.1)  # Points at 0.1m height

                # Stack into Nx3 array
                world_points = np.column_stack((local_x, local_y, local_z))

                # # Transform points to world coordinates
                # world_points = self.transform_robot_to_world(local_points, self.robot_pose)

                colors = np.full((len(world_points), 4), [0.2, 0.2, 0.2, 1.0])  # Dark gray, fully opaque
                radii = np.full(len(world_points), CFG.MAPPING_GRID_GRID_CELL_SIZE / 2)  # Half the cell size

                # Log the occupancy grid in the 'world' frame
                rr.log(
                    "world/occupancy_grid",
                    rr.Points3D(world_points, colors=colors, radii=radii),
                    timeless=False,
                )

            # Convert to a black and white image
            bw_image = (grid_array * 255).astype(np.uint8)
            # Log the image to Rerun
            rr.log("occupancy_grid", rr.Image(bw_image))

    def log_wavemap_occupied_points(self, wavemap_occupied_points_data: WAVEMAP_OCCUPIED_POINTS_MSG):
        if wavemap_occupied_points_data is not None:
            points_np = np.array(wavemap_occupied_points_data.occupied_points)
            d_m = np.linalg.norm(points_np, axis=1)  # distances in meters
            colors = self.cmap(self.norm(d_m))
            radii = np.full(points_np.shape[0], 0.05)

            # Log the points relative to the 'robot' frame
            rr.log(
                "robot/wavemap_occupied_points",
                rr.Points3D(points_np, colors=colors, radii=radii),
                timeless=False,
            )

    def log_tof_map(self, tof_map_data: TOF_MAP_MSG):
        if tof_map_data is not None:
            # Process each sensor's data
            for sensor_data in tof_map_data.sensors:
                sensor_addr = sensor_data["sensor_address"]

                # Process valid points
                valid_points = sensor_data["valid_points"]
                if valid_points:
                    points_np = np.array(valid_points)
                    d_m = np.linalg.norm(points_np, axis=1)  # distances in meters
                    colors = self.cmap(self.norm(d_m))
                    radii = np.full(points_np.shape[0], 0.05)

                    # Log the points relative to the 'robot' frame
                    rr.log(
                        f"robot/tof/sensor_{sensor_addr}/valid",
                        rr.Points3D(points_np, colors=colors, radii=radii),
                        timeless=False,
                    )

                # Process invalid points
                invalid_points = sensor_data["invalid_points"]
                if invalid_points:
                    points_np = np.array(invalid_points)
                    colors = np.full((points_np.shape[0], 4), [1.0, 1.0, 0.0, 0.5])  # Yellow, semi-transparent
                    radii = np.full(points_np.shape[0], 0.05)

                    # Log the points relative to the 'robot' frame
                    rr.log(
                        f"robot/tof/sensor_{sensor_addr}/invalid",
                        rr.Points3D(points_np, colors=colors, radii=radii),
                        timeless=False,
                    )

            # Add occupancy grid visualization
            grid = np.array(tof_map_data.occupancy_grid_data)
            resolution = tof_map_data.occupancy_grid_resolution
            min_x = tof_map_data.occupancy_grid_min_x
            min_y = tof_map_data.occupancy_grid_min_y

            # Create points for occupied cells (where grid == 0)
            occupied_indices = np.argwhere(grid == 0)

            if occupied_indices.size > 0:
                # Convert grid indices to robot-local coordinates
                # Grid indices: row (y), col (x)
                local_x = occupied_indices[:, 1] * resolution + min_x + (resolution / 2)
                local_y = occupied_indices[:, 0] * resolution + min_y + (resolution / 2)
                local_z = np.full_like(local_x, 0.1)  # Points at 0.1m height

                # Stack into Nx3 array
                local_points = np.column_stack((local_x, local_y, local_z))

                # Transform points to world coordinates
                world_points = self.transform_robot_to_world(local_points, self.robot_pose)

                colors = np.full((len(world_points), 4), [0.2, 0.2, 0.2, 1.0])  # Dark gray, fully opaque
                radii = np.full(len(world_points), resolution / 2)  # Half the cell size

                # Log the occupancy grid in the 'world' frame
                rr.log(
                    "world/occupancy_grid",
                    rr.Points3D(world_points, colors=colors, radii=radii),
                    timeless=False,
                )

    def log_odometry(self, pose_msg: EXTENDED_POSE_W_BIAS_MSG):
        if pose_msg is not None:
            self.robot_pose = {
                'x': pose_msg.r_a_b_x,
                'y': pose_msg.r_a_b_y,
                'theta': pose_msg.phi_a_b_z
            }

            rr.log("pose", rr.TextLog(
                f"X: {pose_msg.r_a_b_x:.2f}m\n"
                f"Y: {pose_msg.r_a_b_y:.2f}m\n"
            ))
            rr.log("world_pose", rr.Points2D([pose_msg.r_a_b_x, pose_msg.r_a_b_y]))

            # Update the robot's transform in Rerun
            sin_theta_half = math.sin(self.robot_pose['theta'] / 2.0)
            cos_theta_half = math.cos(self.robot_pose['theta'] / 2.0)
            quat = rr.Quaternion(xyzw=[0.0, 0.0, sin_theta_half, cos_theta_half])

            # Log the transform from 'world' to 'robot'
            rr.log(
                "robot",
                rr.Transform3D(
                    translation=[self.robot_pose['x'], self.robot_pose['y'], 0.0],
                    rotation=quat,
                ),
                timeless=False,
            )

            # Log the robot's visualization (optional)
            robot_center = np.array([[0.0, 0.0, 0.7]])  # Robot is at the origin of its own frame

            # Log the box shape
            rr.log(
                "robot/geometry/extrusion",
                rr.Boxes3D(
                    centers=robot_center,
                    half_sizes=self.robot_half_size,
                    colors=self.robot_color
                ),
                timeless=False,
            )

            # Add capsule base
            rr.log(
                "robot/geometry/base",
                rr.Boxes3D(
                    centers=[[0.0, 0.0, 0.0825]],  # Center at base
                    half_sizes=[[0.25, 0.25, 0.075]],  # Half width/length 0.5/2 = 0.25m, height 0.15/2 = 0.075m
                    colors=self.robot_color
                ),
                timeless=False,
            )
            print(f"Updated robot position: x={self.robot_pose['x']:.2f}, y={self.robot_pose['y']:.2f}, theta={self.robot_pose['theta']:.2f}")

            # Append the current position to the robot path (in world frame)
            self.robot_path.append([self.robot_pose['x'], self.robot_pose['y'], 0.05])  # Z-coordinate is consistent with robot_center

            # Log the robot's path as a LineStrips3D in world frame
            if len(self.robot_path) > 1:
                rr.log(
                    "world/robot_path",
                    rr.LineStrips3D(
                        [np.array(self.robot_path)],
                        colors=[[0.0, 0.0, 1.0, 1.0]],  # Blue color for the path
                        radii=0.01
                    ),
                    timeless=False,
                )

    # def log_robot_path(self, robot_path: list):
    #     if robot_path:
    #         # Parse the path plan message
    #         path_xy = path_data["path_xy"]  # These are already in world coordinates

    #         # Convert path to numpy array for visualization
    #         path_points = np.array([[x, y, 0.1] for x, y in path_xy])  # Set Z to 0.1m

    #         # Log the path plan in the world frame (not robot frame)
    #         if len(path_points) > 1:
    #             rr.log(
    #                 "world/path_plan",  # Changed from "robot/path_plan" to "world/path_plan"
    #                 rr.LineStrips3D(
    #                     [path_points],
    #                     colors=[[0.0, 1.0, 0.0, 1.0]],  # Green path
    #                     radii=[0.02]
    #                 ),
    #                 timeless=False,
    #             )
    #             print(f"Visualized path with {len(path_points)} points")

    def transform_robot_to_world(self, points, robot_pose):
        """
        Transform points from the robot frame to the world frame using the robot's pose.
        """
        x = robot_pose['x']
        y = robot_pose['y']
        theta = robot_pose['theta']

        # Rotation matrix for 2D transformation
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        rotation_matrix = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta,  0],
            [0,         0,          1]
        ])

        # Rotate and translate points
        world_points = points @ rotation_matrix.T
        world_points[:, 0] += x
        world_points[:, 1] += y

        return world_points

def main():
    """
    Main function to create and run the RerunViewer instance.
    """
    viewer = RerunViewer()
    viewer.run()

if __name__ == "__main__":
    main()
