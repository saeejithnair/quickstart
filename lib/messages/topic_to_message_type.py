"""Auto-generated topic to message type mapping from TOML configuration config/bot_quickstart_msgs.toml at 2025-01-31 01:02:13.427047."""

from lib.messages.extended_pose_w_bias_msg import EXTENDED_POSE_W_BIAS_MSG
from lib.messages.gyro_data_msg import GYRO_DATA_MSG
from lib.messages.localization_initialized_msg import LOCALIZATION_INITIALIZED_MSG
from lib.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from lib.messages.raw_imu_data_msg import RAW_IMU_DATA_MSG
from lib.messages.robot_extended_pose_msg import ROBOT_EXTENDED_POSE_MSG
from lib.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from lib.messages.robot_pose_msg import ROBOT_POSE_MSG
from lib.messages.target_point_msg import TARGET_POINT_MSG
from lib.messages.target_velocity_msg import TARGET_VELOCITY_MSG
from lib.messages.trajectory_msg import TRAJECTORY_MSG
from lib.messages.wheel_velocities_data_msg import WHEEL_VELOCITIES_DATA_MSG

# Topic variables
TOPIC_GYRO = '/imu/gyro'
TOPIC_RAW_IMU = '/imu/raw_imu'
TOPIC_RAW_IMU_UNSCALED_BIASED = '/imu/raw_imu_unscaled_biased'
TOPIC_PROCESSED_IMU = '/localization/processed_imu'
TOPIC_WHEEL_VELOCITIES = '/wheel_velocities_mps'
TOPIC_EXTENDED_POSE_W_BIAS = '/localization/extended_pose_state'
TOPIC_LOCALIZATION_INITIALIZED = '/localization/initialized_flag'
TOPIC_ROBOT_POSE = '/localization/robot_pose'
TOPIC_ROBOT_EXTENDED_POSE = '/localization/robot_extended_pose'
TOPIC_ROBOT_POSE_GRID_COORDS = '/mapping/robot_pose_grid_coords'
TOPIC_OCCUPANCY_GRID = '/mapping/occupancy_grid'
TOPIC_TRAVERSABILITY_GRID = '/mapping/traversability_grid'
TOPIC_TRAJECTORY = '/planning/trajectory'
TOPIC_TARGET_POINT = '/planning/target_point'
TOPIC_TARGET_VELOCITY = '/control/target_velocity'

# Topic to message type mapping
topic_to_message_type = {
    '/imu/gyro': GYRO_DATA_MSG,
    '/imu/raw_imu': RAW_IMU_DATA_MSG,
    '/imu/raw_imu_unscaled_biased': RAW_IMU_DATA_MSG,
    '/localization/processed_imu': RAW_IMU_DATA_MSG,
    '/wheel_velocities_mps': WHEEL_VELOCITIES_DATA_MSG,
    '/localization/extended_pose_state': EXTENDED_POSE_W_BIAS_MSG,
    '/localization/initialized_flag': LOCALIZATION_INITIALIZED_MSG,
    '/localization/robot_pose': ROBOT_POSE_MSG,
    '/localization/robot_extended_pose': ROBOT_EXTENDED_POSE_MSG,
    '/mapping/robot_pose_grid_coords': ROBOT_POSE_GRID_COORDS_MSG,
    '/mapping/occupancy_grid': OCCUPANCY_GRID_MSG,
    '/mapping/traversability_grid': OCCUPANCY_GRID_MSG,
    '/planning/trajectory': TRAJECTORY_MSG,
    '/planning/target_point': TARGET_POINT_MSG,
    '/control/target_velocity': TARGET_VELOCITY_MSG,
}
