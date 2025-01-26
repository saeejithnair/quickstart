import json
import logging
import os
import time
import traceback

import numpy as np

import lib.constants as CFG
from lib.control.odrive_uart import ODriveUART, reset_odrive
from lib.data_logger import DataLogger
from lib.logging_utils import Logger

YAW_RATE_TO_MOTOR_TORQUE = (CFG.ROBOT_WHEEL_DIST_M / CFG.ROBOT_WHEEL_RADIUS_M) * 0.1  # Assuming a rough torque constant
MOTOR_TURNS_TO_LINEAR_POS = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi
RPM_TO_METERS_PER_SECOND = CFG.ROBOT_WHEEL_RADIUS_M * 2 * np.pi / 60

class MotorControl:
    def __init__(self, enable_motor_control=True, logger=None):
        """
        Initialize the MotorControl with optional motor control and logger.

        :param enable_motor_control: If True, motor control is enabled.
        :param logger: Optional logger instance.
        """
        if logger is None:
            self.logger = Logger('control', 'logs/control.log', level=logging.INFO)
        else:
            self.logger = logger

        self.enable_motor_control = enable_motor_control

        self.desired_yaw_rate_rad_s = 0
        self.desired_vel_mps = 0

        # For plotting and logging
        self.data_logger = DataLogger()
        self.start_plot_time = time.monotonic()

        try:
            self.left_motor_dir, self.right_motor_dir = get_motor_directions()
        except Exception as e:
            print(f"Error getting motor directions from JSON file, using constants from config instead: {e}")
            self.left_motor_dir, self.right_motor_dir = CFG.MOTOR_CONTROL_LEFT_MOTOR_DIR, CFG.MOTOR_CONTROL_RIGHT_MOTOR_DIR

        # Initialize motors
        self.reset_odrive()
        time.sleep(1)
        self.motor_controller = ODriveUART(CFG.MOTOR_CONTROL_SERIAL_PORT,
                                           left_axis=CFG.MOTOR_CONTROL_LEFT_MOTOR_AXIS,
                                           right_axis=CFG.MOTOR_CONTROL_RIGHT_MOTOR_AXIS,
                                           dir_left=self.left_motor_dir,
                                           dir_right=self.right_motor_dir)
        self.motor_controller.clear_errors_left()
        self.motor_controller.clear_errors_right()
        self.motor_controller.start_left()
        # self.motor_controller.enable_torque_mode_left()
        self.motor_controller.enable_velocity_mode_left()
        self.motor_controller.start_right()
        # self.motor_controller.enable_torque_mode_right()
        self.motor_controller.enable_velocity_mode_right()
        self.motor_controller.set_speed_rpm_left(0)
        self.motor_controller.set_speed_rpm_right(0)

        self.l_vel_mps = 0
        self.r_vel_mps = 0

        # Record starting position
        self.l_pos_m = 0
        self.r_pos_m = 0
        self.start_pos_m = 0
        self.start_yaw_rad = 0
        try:    
            self.l_pos_m = self.motor_controller.get_position_turns_left()
            self.r_pos_m = self.motor_controller.get_position_turns_right()
            self.start_pos_m = (self.l_pos_m + self.r_pos_m) / 2 * MOTOR_TURNS_TO_LINEAR_POS
            self.start_yaw_rad = (self.l_pos_m - self.r_pos_m) * MOTOR_TURNS_TO_LINEAR_POS / (2*CFG.ROBOT_WHEEL_DIST_M)
        except Exception as e:
            print(f"Error reading initial motor positions: {e}")
            self.reset_and_initialize_motors()
            return

        self.cycle_count = 0

    def reset_and_initialize_motors(self):
        """
        Reset the ODrive and re-initialize the motors.
        """
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
            self.motor_controller.start_left()
            # self.motor_controller.enable_torque_mode_left()
            self.motor_controller.enable_velocity_mode_left()
            self.motor_controller.start_right()
            # self.motor_controller.enable_torque_mode_right()
            self.motor_controller.enable_velocity_mode_right()
            print("Motors re-initialized successfully.")
        except Exception as e:
            print(f"Error re-initializing motors: {e}")

    def reset_odrive(self):
        """
        Reset the ODrive controller.
        """
        reset_odrive()
        time.sleep(0.5)

    def control_step(self, velocity_target_mps=0.0, yaw_rate_target_rad_s=0.0):
        """
        Perform a single control step.

        :param velocity_target_mps: Target velocity in meters per second.
        :param yaw_rate_target_rad_s: Target yaw rate in radians per second.
        :return: Tuple of left and right wheel velocities in meters per second.
        """
        try:
            # Get motor data
            try:
                self.l_pos_m, self.l_vel_mps = self.motor_controller.get_pos_vel_left()
                self.r_pos_m, self.r_vel_mps = self.motor_controller.get_pos_vel_right()

                self.l_vel_mps = self.l_vel_mps * RPM_TO_METERS_PER_SECOND
                self.r_vel_mps = self.r_vel_mps * RPM_TO_METERS_PER_SECOND
            except Exception as e:
                print('Motor controller error:', e)
                self.reset_and_initialize_motors()
                return self.l_vel_mps, self.r_vel_mps
            
            # Motor error checks (every 20 cycles)
            if self.cycle_count % 20 == 0:
                try:
                    if self.motor_controller.has_errors():
                        self.motor_controller.dump_errors()
                        self.reset_and_initialize_motors()
                        return self.l_vel_mps, self.r_vel_mps
                except Exception as e:
                    print('Error checking motor errors:', e)
                    self.reset_and_initialize_motors()
                    return self.l_vel_mps, self.r_vel_mps

            self.desired_vel_mps = velocity_target_mps
            self.desired_yaw_rate_rad_s = yaw_rate_target_rad_s

            # Calculate left and right wheel velocities
            wheel_base_width_m = CFG.ROBOT_WHEEL_DIST_M
            left_wheel_velocity = self.desired_vel_mps - (wheel_base_width_m * self.desired_yaw_rate_rad_s) / 2
            right_wheel_velocity = self.desired_vel_mps + (wheel_base_width_m * self.desired_yaw_rate_rad_s) / 2

            # Convert velocities to RPM for motor control
            left_wheel_rpm = left_wheel_velocity / RPM_TO_METERS_PER_SECOND
            right_wheel_rpm = right_wheel_velocity / RPM_TO_METERS_PER_SECOND

            # Set motor speeds
            if self.enable_motor_control:
                try:
                    self.motor_controller.set_speed_rpm_left(left_wheel_rpm)
                    self.motor_controller.set_speed_rpm_right(right_wheel_rpm)
                except Exception as e:
                    print('Motor controller error:', e)
                    self.reset_and_initialize_motors()
                    return self.l_vel_mps, self.r_vel_mps

            self.cycle_count += 1

        except Exception as e:
            print("Balance step error:", e)
            traceback.print_exc()

        return self.l_vel_mps, self.r_vel_mps
    
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
