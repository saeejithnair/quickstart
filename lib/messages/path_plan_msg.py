"""Auto-generated MQTT message class from TOML configuration config/bot_quickstart_msgs.toml at 2025-02-01 21:01:24.899604."""

import json

from lib.messages.mqtt_message_base import MqttMessageBase


class PATH_PLAN_MSG(MqttMessageBase):
    """MQTT message class for PATH_PLAN."""

    timestamp: float = None
    path_pose_list: list = None

    def __init__(self, timestamp: float | None = None, path_pose_list: list | None = None):
        """Initialize the message class with given fields."""
        self.timestamp = timestamp
        self.path_pose_list = path_pose_list

    def convert_to_payload(self) -> str:
        """Convert the message fields to a JSON payload."""
        try:
            data = {
                'timestamp': self.timestamp,
                'path_pose_list': self.path_pose_list,
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
            self.path_pose_list = data['path_pose_list']
        except (json.JSONDecodeError, KeyError) as e:
            raise Exception(f'Error converting from payload: {e}') from e
