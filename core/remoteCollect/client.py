from websockets.sync.client import connect
from pynput import keyboard
import time

addr = "192.168.0.105:8765"

last_sent = None
last_time = 0
throttle_delay = 0.1 # ms

def send(msg):
    global last_sent, last_time
    now = time.time()
    if msg != last_sent or (now - last_time) > throttle_delay:
        with connect(f"ws://{addr}") as websocket:
            websocket.send(msg)
        last_sent = msg
        last_time = now

def on_press(key):
    try:
        if key.char == 'a':
            send("left")
        elif key.char == 's':
            send("down")
        elif key.char == 'd':
            send("right")
        elif key.char == 'w':
            send("up")
        elif key.char == 'q':
            send("quit")
    except AttributeError:
        pass

def on_release(key):
    send("stop")

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)

listener.start()
listener.join()
print("hello world")
