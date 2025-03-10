# Adds the lib directory to the Python path
import sys
import os
import math
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import json
from lib.odrive_uart import ODriveUART

class RobotDriver:
    def __init__(self, motor):
        self.motor = motor
        self.wheel_diameter = 0.165  # 165mm in meters
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.wheel_distance = 0.4  # Distance between wheels in meters (adjust as needed)
        
        # Starting position
        self.left_start_pos = 0
        self.right_start_pos = 0
    
    def update_start_position(self):
        """Update the starting position for distance calculations"""
        # Get current position in turns
        self.left_start_pos = self.motor.get_position_turns_left()
        self.right_start_pos = self.motor.get_position_turns_right()
    
    def drive_distance(self, distance, speed=0.2):
        """
        Drive straight for a specified distance
        
        Args:
            distance: Distance to drive in meters
            speed: Speed in m/s
        """
        print(f"Driving {distance}m at {speed}m/s")
        
        # Update starting position
        self.update_start_position()
        
        # Calculate required wheel rotations
        required_rotations = distance / self.wheel_circumference
        
        # Set motor speeds
        self.motor.set_speed_mps_left(speed)
        self.motor.set_speed_mps_right(speed)
        
        # Monitor distance traveled
        while True:
            # Get current positions
            left_pos = self.motor.get_position_turns_left()
            right_pos = self.motor.get_position_turns_right()
            
            # Calculate rotations since start
            left_rotations = abs(left_pos - self.left_start_pos)
            right_rotations = abs(right_pos - self.right_start_pos)
            
            # Use average of both wheels
            avg_rotations = (left_rotations + right_rotations) / 2
            
            if avg_rotations >= required_rotations:
                break
                
            time.sleep(0.01)
        
        # Stop motors
        self.motor.set_speed_mps_left(0)
        self.motor.set_speed_mps_right(0)
        print(f"Completed driving {distance}m")
    
    def turn_degrees(self, degrees, turn_speed=0.2):
        """
        Turn the robot by a specified angle
        
        Args:
            degrees: Angle to turn in degrees (positive = right/clockwise, negative = left/counterclockwise)
            turn_speed: Speed for turning in m/s
        """
        print(f"Turning {degrees} degrees")
        
        # Update starting position
        self.update_start_position()
        
        # Calculate the arc length for the turn
        # For a differential drive robot turning in place, each wheel travels in opposite directions
        # The arc length is (angle in radians) * (wheel distance / 2)
        angle_radians = math.radians(abs(degrees))
        arc_length = angle_radians * (self.wheel_distance / 2)
        
        # Calculate required wheel rotations
        required_rotations = arc_length / self.wheel_circumference
        
        # Set motor speeds based on turn direction
        if degrees > 0:  # Turn right/clockwise
            self.motor.set_speed_mps_left(turn_speed)
            self.motor.set_speed_mps_right(-turn_speed)
        else:  # Turn left/counterclockwise
            self.motor.set_speed_mps_left(-turn_speed)
            self.motor.set_speed_mps_right(turn_speed)
        
        # Monitor rotation
        while True:
            # Get current positions
            left_pos = self.motor.get_position_turns_left()
            right_pos = self.motor.get_position_turns_right()
            
            # Calculate rotations since start (use absolute value)
            left_rotations = abs(left_pos - self.left_start_pos)
            right_rotations = abs(right_pos - self.right_start_pos)
            
            # Use average of both wheels
            avg_rotations = (left_rotations + right_rotations) / 2
            
            if avg_rotations >= required_rotations:
                break
                
            time.sleep(0.01)
        
        # Stop motors
        self.motor.set_speed_mps_left(0)
        self.motor.set_speed_mps_right(0)
        print(f"Completed turning {degrees} degrees")
    
    def drive_circle(self, radius=0.5, speed=0.2, duration=10):
        """
        Drive in a circle with specified radius, speed, and duration.
        
        Args:
            radius: Circle radius in meters
            speed: Linear speed in m/s
            duration: Duration in seconds
        """
        print(f"Driving in a circle: radius={radius}m, speed={speed}m/s, duration={duration}s")
        
        # Calculate wheel speeds for circular motion
        inner_speed = speed * (radius - self.wheel_distance/2) / radius
        outer_speed = speed * (radius + self.wheel_distance/2) / radius
        
        start_time = time.time()
        while time.time() - start_time < duration:
            # For right turn (clockwise)
            self.motor.set_speed_mps_left(outer_speed)
            self.motor.set_speed_mps_right(inner_speed)
            time.sleep(0.01)
        
        # Stop after completing the circle
        self.motor.set_speed_mps_left(0)
        self.motor.set_speed_mps_right(0)
        print("Circle completed")
    
    def drive_square(self, side_length=1.0, speed=0.2):
        """
        Drive in a square with specified side length and speed using helper functions.
        
        Args:
            side_length: Length of each side in meters
            speed: Linear speed in m/s
        """
        print(f"Driving in a square: side_length={side_length}m, speed={speed}m/s")
        
        for i in range(4):
            print(f"Side {i+1}")
            # Drive one side
            self.drive_distance(side_length, speed)
            time.sleep(0.5)  # Pause briefly
            
            # Turn 90 degrees right
            self.turn_degrees(90, speed/2)
            time.sleep(0.5)  # Pause briefly
        
        print("Square completed")

def main():
    # Load motor direction configuration
    try:
        with open(os.path.expanduser('~/quickstart/lib/motor_dir.json'), 'r') as f:
            motor_dirs = json.load(f)
    except Exception as e:
        print(f"Error reading motor_dir.json: {e}")
        return
    
    # Initialize motor controller
    motor = ODriveUART(port='/dev/ttyAMA1', left_axis=0, right_axis=1, 
                       dir_left=motor_dirs['left'], dir_right=motor_dirs['right'])
    
    # Start motors and set to velocity mode
    motor.start_left()
    motor.start_right()
    motor.enable_velocity_mode_left()
    motor.enable_velocity_mode_right()
    motor.disable_watchdog_left()
    motor.disable_watchdog_right()
    motor.clear_errors_left()
    motor.clear_errors_right()
    
    # Create robot driver
    robot = RobotDriver(motor)
    
    try:
        # Test the helper functions
        print("Testing drive_distance...")
        robot.drive_distance(0.5, 0.2)  # Drive 0.5m at 0.2m/s
        time.sleep(1)
        
        print("Testing turn_degrees...")
        robot.turn_degrees(90, 0.2)  # Turn 90 degrees right
        time.sleep(1)
        robot.turn_degrees(-90, 0.2)  # Turn 90 degrees left
        time.sleep(1)
        
        # First drive in a circle
        robot.drive_circle(radius=0.1, speed=0.2, duration=10)
        time.sleep(2)
        
        # Then drive in a square
        robot.drive_square(side_length=0.5, speed=0.2)
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Always stop motors and clean up
        print("Stopping motors")
        motor.set_speed_mps_left(0)
        motor.set_speed_mps_right(0)

if __name__ == "__main__":
    main()

