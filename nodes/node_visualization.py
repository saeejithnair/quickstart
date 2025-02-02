import logging

import pygame

from lib.logging_utils import Logger
from lib.messages.mqtt_utils import MQTTSubscriber, MQTTPublisher
from lib.messages.occupancy_grid_msg import OCCUPANCY_GRID_MSG
from lib.messages.target_point_msg import TARGET_POINT_MSG
from lib.messages.robot_pose_grid_coords_msg import ROBOT_POSE_GRID_COORDS_MSG
from lib.messages.topic_to_message_type import TOPIC_OCCUPANCY_GRID, TOPIC_TARGET_POINT, TOPIC_ROBOT_POSE_GRID_COORDS

class VisualizationNode(object):
    """VisualizationNode class."""

    def __init__(self, logging_level=logging.INFO):
        self.logger = Logger('visualization', 'logs/visualization.log', level=logging_level)

        self.mqtt_subscriber = MQTTSubscriber(broker_address="localhost", topic_to_message_map={TOPIC_OCCUPANCY_GRID: OCCUPANCY_GRID_MSG,
                                                                                                TOPIC_ROBOT_POSE_GRID_COORDS: ROBOT_POSE_GRID_COORDS_MSG})
        self.mqtt_publisher = MQTTPublisher(broker_address="localhost", topic_to_message_map={TOPIC_TARGET_POINT: TARGET_POINT_MSG})

        self.occupancy_grid_msg = None
        self.robot_pose_grid_coords_msg = None

    def run(self):
        self.mqtt_subscriber.start()
        self.mqtt_publisher.run()

        pygame.init()
        screen = pygame.display.set_mode((800, 600))

        try:
            while True:
                self.occupancy_grid_msg = self.mqtt_subscriber.get_latest_message(TOPIC_OCCUPANCY_GRID)
                self.robot_pose_grid_coords_msg = self.mqtt_subscriber.get_latest_message(TOPIC_ROBOT_POSE_GRID_COORDS)

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        break
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        x, y = pygame.mouse.get_pos()
                        x_grid = x // 10
                        y_grid = y // 10
                        target_point_msg = TARGET_POINT_MSG()
                        target_point_msg.x_grid = x_grid
                        target_point_msg.y_grid = y_grid
                        self.mqtt_publisher.publish(TOPIC_TARGET_POINT, target_point_msg)

                self.draw_grid(screen)
                pygame.display.flip()
        except KeyboardInterrupt:
            self.logger.info("Visualization node stopped by user.")
        finally:
            self.mqtt_subscriber.stop()
            self.mqtt_publisher.stop()
            pygame.quit()

    def draw_grid(self, screen):
        if self.occupancy_grid_msg is not None:
            width = self.occupancy_grid_msg.width
            grid = self.occupancy_grid_msg.flattened_grid_list

            for y in range(width):
                for x in range(width):
                    value = grid[y * width + x]
                    color = (255, 255, 255) if value == 0 else (0, 0, 0)
                    pygame.draw.rect(screen, color, pygame.Rect(x * 10, y * 10, 10, 10))

        if self.robot_pose_grid_coords_msg is not None:
            x = self.robot_pose_grid_coords_msg.x_grid
            y = self.robot_pose_grid_coords_msg.y_grid
            pygame.draw.circle(screen, (0, 0, 255), (x * 10, y * 10), 5)

if __name__ == "__main__":
    visualization_node = VisualizationNode()
    visualization_node.run()
