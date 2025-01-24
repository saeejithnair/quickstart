# Adds the lib directory to the Python path
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from examples.drive_controller import RobotController

def test_wiggle():
    # Initialize robot controller
    robot = RobotController()
    
    try:
        # Turn 360 degrees clockwise
        robot.turn_degrees(720)
        
    finally:
        # Cleanup and stop motors
        robot.cleanup()

if __name__ == "__main__":
    test_wiggle()
