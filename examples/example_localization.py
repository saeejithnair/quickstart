#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import tomlkit
import numpy as np
import rerun as rr
from lib.odrive_uart import ODriveUART
from lib.imu import FilteredMPU6050
from lib.localization import RobotEKF

def main():
    # Initialize Rerun (don't spawn viewer on Pi)
    rr.init("robot_localization", spawn=False)
    
    # Connect to the Rerun server running on your computer using the new TCP method
    print("Connecting to Rerun viewer...")
    rr.connect_tcp("192.168.2.24:9876")  # Adjust IP and port as needed

    # Set up coordinate system and views
    # rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN)
    
    # Set up plot styling (static, won't change over time)
    rr.log("plots/velocities/linear", 
           rr.SeriesLine(color=[0, 255, 0], name="Linear Velocity (m/s)"),
           static=True)
    rr.log("plots/velocities/angular/encoder", 
           rr.SeriesLine(color=[255, 0, 0], name="Angular Velocity - Encoder (rad/s)"),
           static=True)
    
    # IMU plots in separate window
    rr.log("imu_plots/angular_velocity/x", 
           rr.SeriesLine(color=[0, 0, 255], name="X-axis (rad/s)"),
           static=True)
    rr.log("imu_plots/angular_velocity/y", 
           rr.SeriesLine(color=[0, 255, 255], name="Y-axis (rad/s)"),
           static=True)
    rr.log("imu_plots/angular_velocity/z", 
           rr.SeriesLine(color=[255, 0, 255], name="Z-axis (rad/s)"),
           static=True)

    # Initialize sensors
    try:
        with open('../config/motor.toml', 'r') as f:  # Fixed path to motor.toml
            motor_config = tomlkit.load(f)
            left_dir = motor_config['motor_directions']['left']
            right_dir = motor_config['motor_directions']['right']
    except Exception as e:
        print("Error reading motor.toml:", e)
        return

    # Initialize ODrive
    print("Initializing ODrive...")
    odrive = ODriveUART(port='/dev/ttyAMA1', 
                       left_axis=0, 
                       right_axis=1, 
                       dir_left=left_dir, 
                       dir_right=right_dir)
    
    # Initialize IMU
    print("Initializing IMU...")
    imu = FilteredMPU6050()
    imu.calibrate()
    
    # Initialize EKF
    ekf = RobotEKF(dt=0.02)  # 50Hz update rate
    
    print("Starting sensor fusion...")
    start_time = time.monotonic()
    
    try:
        while time.monotonic() - start_time < 30:  # Run for 30 seconds
            # Get encoder data (average of left and right wheels for v)
            v_left = odrive.get_speed_rpm_left() / 60.0  # Convert RPM to RPS
            v_right = odrive.get_speed_rpm_right() / 60.0
            v_avg = (v_left + v_right) / 2.0  # Average forward velocity
            w_enc = (v_right - v_left) / 2.0  # Differential drive kinematics
            
            # Get IMU data
            _, _, yaw = imu.get_orientation()  # We mainly care about yaw
            w_imu_x, w_imu_y, w_imu_z = imu.gyro  # Get all angular velocities from gyro
            
            # EKF prediction and update
            ekf.predict()
            ekf.update(v_encoder=v_avg, w_encoder=w_enc, w_imu=w_imu_z)  # Use z-axis for yaw
            
            # Get current state estimates
            x, y, theta = ekf.get_pose()
            v, w = ekf.get_velocity()
            
            # Log robot pose and path
            t = time.monotonic() - start_time
            
            # Log robot position and orientation
            rr.log("world/robot", 
                  rr.Transform3D(
                      translation=[x, y, 0.0],
                      rotation=rr.RotationAxisAngle(axis=[0, 0, 1], angle=theta),
                  ))
            
            # Log robot base box (0.5m wide x 0.2m thick x 0.2m tall)
            rr.log("world/robot/base",
                  rr.Boxes3D(
                      half_sizes=[[0.1, 0.25, 0.1]],  # x=thickness/2, y=width/2, z=height/2
                      centers=[[0.0, 0.0, 0.1]],  # Raise in Z direction
                      colors=[[100, 100, 100]],
                  ))
            
            # Log robot stick (0.05m x 0.05m x 1.5m tall)
            rr.log("world/robot/stick",
                  rr.Boxes3D(
                      half_sizes=[[0.025, 0.025, 0.75]],  # x,y=thickness/2, z=height/2
                      centers=[[0.0, 0.0, 0.85]],  # Center at half stick height (0.75) + base height (0.1)
                      colors=[[150, 75, 0]],
                  ))
            
            # Log robot direction arrow (smaller now since we have the boxes)
            rr.log("world/robot/direction", 
                  rr.Arrows3D(
                      vectors=[[0.2, 0.0, 0.0]],  # Point in X direction (forward)
                      origins=[[0.0, 0.0, 0.2]],  # Place arrow on top of base
                      colors=[0, 255, 0],
                  ))
            
            # Log path history at base height
            rr.log("world/path",
                  rr.Points3D(
                      positions=[[x, y, 0.1]],  # At half height of base
                      colors=[0, 0, 255],
                      radii=[0.02],  # Make points smaller
                  ))
            
            # Log velocities as timeseries
            rr.log("plots/velocities/linear", 
                  rr.Scalar(v_avg))
            rr.log("plots/velocities/angular/encoder", 
                  rr.Scalar(w_enc))
            
            # Log IMU data in separate window
            rr.log("imu_plots/angular_velocity/x", 
                  rr.Scalar(w_imu_x))
            rr.log("imu_plots/angular_velocity/y", 
                  rr.Scalar(w_imu_y))
            rr.log("imu_plots/angular_velocity/z", 
                  rr.Scalar(w_imu_z))
            
            # Print current state
            print(f"\rTime: {t:.1f}s | "
                  f"Pose: x={x:.2f}, y={y:.2f}, θ={np.degrees(theta):.1f}° | "
                  f"Velocity: v={v:.2f}, ω={w:.2f}", end='')
            
            # Control loop rate
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Stop motors
        odrive.stop_left()
        odrive.stop_right()

if __name__ == '__main__':
    main()
