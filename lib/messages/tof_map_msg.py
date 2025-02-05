"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class TOF_MAP_MSG(MqttMessageBase):
    """MQTT message class for TOF_MAP."""

    timestamp: float = None
    sensors: list = None
    occupancy_grid_data: list = None
    occupancy_grid_height: int = None
    occupancy_grid_width: int = None
    occupancy_grid_resolution: float = None
    occupancy_grid_min_x: float = None
    occupancy_grid_max_x: float = None
    occupancy_grid_min_y: float = None
    occupancy_grid_max_y: float = None

    def __init__(self, timestamp: float | None = None, sensors: list | None = None, occupancy_grid_data: list | None = None, occupancy_grid_height: int | None = None, occupancy_grid_width: int | None = None, occupancy_grid_resolution: float | None = None, occupancy_grid_min_x: float | None = None, occupancy_grid_max_x: float | None = None, occupancy_grid_min_y: float | None = None, occupancy_grid_max_y: float | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.sensors = sensors
        self.occupancy_grid_data = occupancy_grid_data
        self.occupancy_grid_height = occupancy_grid_height
        self.occupancy_grid_width = occupancy_grid_width
        self.occupancy_grid_resolution = occupancy_grid_resolution
        self.occupancy_grid_min_x = occupancy_grid_min_x
        self.occupancy_grid_max_x = occupancy_grid_max_x
        self.occupancy_grid_min_y = occupancy_grid_min_y
        self.occupancy_grid_max_y = occupancy_grid_max_y

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'sensors': self.sensors,
                'occupancy_grid_data': self.occupancy_grid_data,
                'occupancy_grid_height': self.occupancy_grid_height,
                'occupancy_grid_width': self.occupancy_grid_width,
                'occupancy_grid_resolution': self.occupancy_grid_resolution,
                'occupancy_grid_min_x': self.occupancy_grid_min_x,
                'occupancy_grid_max_x': self.occupancy_grid_max_x,
                'occupancy_grid_min_y': self.occupancy_grid_min_y,
                'occupancy_grid_max_y': self.occupancy_grid_max_y,
            }
            return json.dumps(data)
        except (TypeError, ValueError) as e:
            raise Exception(f'Error converting to payload: {e}') from e

    def convert_to_message(self, payload):
        """Convert a JSON payload to message fields."""
        try:
            data = json.loads(payload)
            if 'data' in data:
                data = data['data']
            self.timestamp = data['timestamp']
            self.sensors = data['sensors']
            self.occupancy_grid_data = data['occupancy_grid_data']
            self.occupancy_grid_height = data['occupancy_grid_height']
            self.occupancy_grid_width = data['occupancy_grid_width']
            self.occupancy_grid_resolution = data['occupancy_grid_resolution']
            self.occupancy_grid_min_x = data['occupancy_grid_min_x']
            self.occupancy_grid_max_x = data['occupancy_grid_max_x']
            self.occupancy_grid_min_y = data['occupancy_grid_min_y']
            self.occupancy_grid_max_y = data['occupancy_grid_max_y']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
