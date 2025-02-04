import json
import logging
import os
import time
import traceback

import numpy as np

import lib.constants as CFG
from lib.control.odrive_uart import ODriveUART, reset_odrive
from lib.logging_utils import Logger

YAW_RATE_TO_MOTOR_TORQUE = (CFG.ROBOT_WHEEL_DIST_M / CFG.ROBOT_WHEEL_RADIUS_M) * 0.1  # Assuming a rough torque constant
MOTOR_TURNS_TO_LINEAR_POS = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi
RPM_TO_METERS_PER_SECOND = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi / 60

class MotorControl:
    def __init__(self, logger=None):
        """
        Initialize the MotorControl with optional motor control and logger.

        :param logger: Optional logger instance.
        """
        if logger is None:
            self.logger = Logger('control', 'logs/control.log', level=logging.INFO)
        else:
            self.logger = logger

        try:
            self.left_motor_dir, self.right_motor_dir = get_motor_directions()
        except Exception as e:
            print(f"Error getting motor directions from JSON file, using constants from config instead: {e}")
            self.left_motor_dir, self.right_motor_dir = CFG.MOTOR_CONTROL_LEFT_MOTOR_DIR, CFG.MOTOR_CONTROL_RIGHT_MOTOR_DIR

        # Initialize motors
        self.reset_and_initialize_motors()

        self.cycle_count = 0

    def start_motors_velocity_mode(self):
        """Initialize and start motors in velocity mode"""
        self.motor_controller.start_left()
        self.motor_controller.start_right()
        self.motor_controller.enable_velocity_mode_left()
        self.motor_controller.enable_velocity_mode_right()

    def start_motors_torque_mode(self):
        """Initialize and start motors in torque mode"""
        self.motor_controller.start_left()
        self.motor_controller.start_right()
        self.motor_controller.enable_torque_mode_left()
        self.motor_controller.enable_torque_mode_right()

    def reset_and_initialize_motors(self, torque_mode=False):
        """Reset the ODrive and re-initialize the motors."""
        self.reset_odrive()
        time.sleep(1)  # Give ODrive time to reset
        try:
            self.motor_controller = ODriveUART(CFG.MOTOR_CONTROL_SERIAL_PORT,
                                               left_axis=CFG.MOTOR_CONTROL_LEFT_MOTOR_AXIS,
                                               right_axis=CFG.MOTOR_CONTROL_RIGHT_MOTOR_AXIS,
                                               dir_left=self.left_motor_dir,
                                               dir_right=self.right_motor_dir)
            self.motor_controller.clear_errors_left()
            self.motor_controller.clear_errors_right()
            if torque_mode:
                self.start_motors_torque_mode()
            else:
                self.start_motors_velocity_mode()
            print("Motors re-initialized successfully.")
        except Exception as e:
            print(f"Error re-initializing motors: {e}")

    def reset_odrive(self):
        """
        Reset the ODrive controller.
        """
        reset_odrive()
        time.sleep(0.5)

    def get_left_motor_velocity(self):
        """Get the current velocity of the left motor."""
        try:
            l_pos_m, l_vel_mps = self.motor_controller.get_pos_vel_left()

            l_vel_mps = l_vel_mps * RPM_TO_METERS_PER_SECOND

            return l_vel_mps
        except Exception as e:
            print('Motor controller error:', e)
            self.reset_and_initialize_motors()
            return 0.0
    
    def get_right_motor_velocity(self):
        """Get the current velocity of the right motor."""
        try:
            r_pos_m, r_vel_mps = self.motor_controller.get_pos_vel_right()

            r_vel_mps = r_vel_mps * RPM_TO_METERS_PER_SECOND

            return r_vel_mps
        except Exception as e:
            print('Motor controller error:', e)
            self.reset_and_initialize_motors()
            return 0.0

    def set_linear_angular_velocities(self, velocity_target_mps=0.0, yaw_rate_target_rad_s=0.0):
        """
        Set the linear and angular velocities of the robot.

        :param velocity_target_mps: Target velocity in meters per second.
        :param yaw_rate_target_rad_s: Target yaw rate in radians per second.
        """
        try:
            # Motor error checks (every 20 cycles)
            if self.cycle_count % 20 == 0:
                try:
                    if self.motor_controller.has_errors():
                        self.motor_controller.dump_errors()
                        self.reset_and_initialize_motors()
                        return
                except Exception as e:
                    print('Error checking motor errors:', e)
                    self.reset_and_initialize_motors()
                    return
                
            # Clip the desired velocity to the maximum speed
            velocity_target_mps = max(min(velocity_target_mps, CFG.MOTOR_CONTROL_MAX_SPEED_LINEAR_MPS), -CFG.MOTOR_CONTROL_MAX_SPEED_LINEAR_MPS)
            yaw_rate_target_rad_s = max(min(yaw_rate_target_rad_s, CFG.MOTOR_CONTROL_MAX_SPEED_ANGULAR_RADPS), -CFG.MOTOR_CONTROL_MAX_SPEED_ANGULAR_RADPS)

            # Calculate left and right wheel velocities
            wheel_base_width_m = CFG.ROBOT_WHEEL_DIST_M
            left_wheel_velocity = velocity_target_mps - (wheel_base_width_m * yaw_rate_target_rad_s) / 2
            right_wheel_velocity = velocity_target_mps + (wheel_base_width_m * yaw_rate_target_rad_s) / 2

            # Convert velocities to RPM for motor control
            left_wheel_rpm = left_wheel_velocity / RPM_TO_METERS_PER_SECOND
            right_wheel_rpm = right_wheel_velocity / RPM_TO_METERS_PER_SECOND

            # Set motor speeds
            try:
                self.motor_controller.set_speed_rpm_left(left_wheel_rpm)
                self.motor_controller.set_speed_rpm_right(right_wheel_rpm)
            except Exception as e:
                print('Motor controller error:', e)
                self.reset_and_initialize_motors()
                return

            self.cycle_count += 1

        except Exception as e:
            print("Balance step error:", e)
            traceback.print_exc()

    def drive_distance(self, distance_meters):
        """Drive forward/backward a specified distance in meters"""
        # Calculate target position
        target_rotations = distance_meters / CFG.ROBOT_WHEEL_RADIUS_M * 2
        initial_pos = self.motor_controller.get_position_turns_left()
        target_pos = initial_pos + target_rotations
        
        # Accelerate
        current_speed = 0.0
        start_time = time.time()
        while current_speed < CFG.MOTOR_CONTROL_MAX_SPEED_LINEAR_MPS:
            elapsed = time.time() - start_time
            current_speed = min(0.15 * elapsed, CFG.MOTOR_CONTROL_MAX_SPEED_LINEAR_MPS)
            self.motor_controller.set_speed_mps_left(current_speed)
            self.motor_controller.set_speed_mps_right(current_speed)
            time.sleep(0.01)

        # Monitor position until target
        while True:
            current_pos = self.motor_controller.get_position_turns_left()
            distance_remaining = (target_pos - current_pos) * CFG.ROBOT_WHEEL_RADIUS_M * 2
            
            if distance_remaining < 0.15:
                current_speed = max(0.05, (distance_remaining / 0.15) * CFG.MOTOR_CONTROL_MAX_SPEED_LINEAR_MPS)
                self.motor_controller.set_speed_mps_left(current_speed)
                self.motor_controller.set_speed_mps_right(current_speed)
                
            if current_pos >= target_pos:
                break
            time.sleep(0.01)

        self.stop()

    def stop(self):
        """
        Stop the motors and disable the watchdog.
        """
        self.motor_controller.disable_watchdog_left()
        self.motor_controller.disable_watchdog_right()
        self.motor_controller.stop_left()
        self.motor_controller.stop_right()

def get_motor_directions():
    """
    Retrieve motor directions from a JSON file.

    :return: Tuple of left and right motor directions.
    :raises Exception: If there is an error reading the JSON file.
    """
    try:
        # json file is in the same directory as the script
        with open(os.path.join(os.path.dirname(__file__), 'motor_dir.json'), 'r') as f:
            motor_dir = json.load(f)
        return motor_dir['left'], motor_dir['right']
    except Exception as e:
        raise Exception(f"Error reading motor directions: {e}")
