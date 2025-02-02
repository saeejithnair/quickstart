"""Node for point to point planning."""

import logging
import time

import numpy as np

import lib.constants as CFG
from lib.logging_utils import Logger
from lib.messages.mqtt_utils import MQTTPublisher, MQTTSubscriber
from lib.messages.path_plan_msg import PATH_PLAN_MSG
from lib.messages.robot_pose_msg import ROBOT_POSE_MSG
from lib.messages.target_point_msg import TARGET_POINT_MSG
from lib.messages.topic_to_message_type import (TOPIC_PATH_PLAN,
                                                TOPIC_ROBOT_POSE,
                                                TOPIC_TARGET_POINT)

"""
To use this node, the following nodes need to be launched:

1. node_control.py
2. node_imu.py
3. node_localization.py
4. node_point_to_point.py

And then to send a target point to the node, the following command can be used:

mosquitto_pub -t /planning/target_point -m "{\"timestamp\": 100, \"x_grid\": 0.0, \"y_grid\": 1.0}"
"""

class PointToPointNode(object):
    """PointToPointNode class."""

    def __init__(self, logging_level=logging.INFO):
        """Initialize the PointToPointNode class."""
        self.logger = Logger('point_to_point', 'logs/point_to_point.log', level=logging_level)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_TARGET_POINT: TARGET_POINT_MSG,
                                                                                                TOPIC_ROBOT_POSE: ROBOT_POSE_MSG})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_PATH_PLAN: PATH_PLAN_MSG})

        # Initialize message variables
        self.path_plan_msg = None
        self.robot_pose_msg = None
        self.target_point_msg = None

        self.loop_rate_hz = 30

        self.prev_goal_pose = None
        self.path_pose_list = None

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        replan_time = time.time()  # Initialize replan time

        try:
            while True:
                start_time = time.time()

                # Get robot pose
                self.robot_pose_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE)
                if self.robot_pose_msg is not None:
                    start_pose_x = self.robot_pose_msg.x_m
                    start_pose_y = self.robot_pose_msg.y_m
                else:
                    start_pose_x = None
                    start_pose_y = None

                # Get target point message
                target_point_msg = self.mqtt_subscriber.get_latest_message(TOPIC_TARGET_POINT)
                if target_point_msg is not None:
                    self.target_point_msg = target_point_msg

                if self.target_point_msg is not None:
                    try:
                        goal_pose_x = self.target_point_msg.x_grid
                        goal_pose_y = self.target_point_msg.y_grid
                    except:
                        print("Target point is not in the correct format", self.target_point_msg)
                        continue

                    # If within tolerance of the target point, don't replan
                    if start_pose_x is not None and start_pose_y is not None:
                        if np.linalg.norm([start_pose_x - goal_pose_x, start_pose_y - goal_pose_y]) < 0.15:
                            self.target_point_msg = None
                            self.path_pose_list = None

                            print("Target point has been reached, not replanning.")
                            continue

                # Check if it's time to replan
                if time.time() - replan_time > 10.0 and self.target_point_msg is not None and self.robot_pose_msg is not None:

                    print(f"Attempting to plan from ({start_pose_x}, {start_pose_y}) to ({goal_pose_x}, {goal_pose_y})")

                    self.path_pose_list = [(start_pose_x, start_pose_y), (goal_pose_x, goal_pose_y)]

                    replan_time = time.time()

                # Publish path plan message
                if self.path_pose_list is not None:
                    self.path_plan_msg = PATH_PLAN_MSG()
                    self.path_plan_msg.timestamp = time.time()
                    self.path_plan_msg.path_pose_list = self.path_pose_list
                    self.mqtt_publisher.publish_msg(TOPIC_PATH_PLAN, self.path_plan_msg)

                end_time = time.time()
                sleep_time = 1/self.loop_rate_hz - (end_time - start_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            self.logger.info("Planning node stopped by user.")
        finally:
            self.mqtt_subscriber.stop()
            self.mqtt_publisher.stop()


if __name__ == "__main__":
    point_to_point_node = PointToPointNode()
    point_to_point_node.run()
