"""Base class for all MQTT messages."""

import json


class MqttMessageBase(object):
    """Base class for all MQTT messages."""

    def __init__(self):
        self.default_msg_dict: dict = None

    def convert_to_payload(self) -> str:
        """Convert the message to a payload string."""
        try:
            return json.dumps(self.default_msg_dict)
        except Exception as e:
            raise e

    def convert_to_message(self, payload):
        """Convert the payload to a message."""
        try:
            self.default_msg_dict = json.loads(payload)
        except Exception as e:
            raise e
