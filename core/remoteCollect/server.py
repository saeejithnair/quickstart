import asyncio
from websockets.asyncio.server import serve
import paho.mqtt.client as mqtt

shutdown_event = asyncio.Event()

MQTT_BROKER_ADDRESS = "localhost"
MQTT_TOPIC = "robot/drive"

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.connect(MQTT_BROKER_ADDRESS)
client.loop_start()

async def echo(websocket):
    async for message in websocket:
        if message == "up":
            client.publish(MQTT_TOPIC, "forward")
        elif message == "down":
            client.publish(MQTT_TOPIC, "back")
        elif message == "right":
            client.publish(MQTT_TOPIC, "right")
        elif message == "left":
            client.publish(MQTT_TOPIC, "left")
        elif message == "quit":
            print("Shutdown signal received.")
            shutdown_event.set()
            client.publish(MQTT_TOPIC, "stop")
            client.loop_stop()
            client.disconnect()
            break
        elif message == "stop":
            print("stop")
            client.publish(MQTT_TOPIC, "stop")


async def main():
    async with serve(echo, "0.0.0.0", 8765) as server:
        print("server started")
        await server.serve_forever()

asyncio.run(main())
