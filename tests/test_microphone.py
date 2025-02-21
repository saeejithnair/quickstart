#!/usr/bin/env python3

# Add these lines at the start to suppress ALSA errors
import ctypes
import contextlib

# Load the error handler
ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                     ctypes.c_char_p, ctypes.c_int,
                                     ctypes.c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

@contextlib.contextmanager
def noalsaerr():
    asound = ctypes.CDLL('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)

import pyaudio
import wave
import sys
import time
import tomli
import tomli_w
from pathlib import Path

def select_microphone(p):
    # Print available audio devices
    print("\nAvailable Audio Input Devices:")
    input_devices = []
    for i in range(p.get_device_count()):
        dev_info = p.get_device_info_by_index(i)
        if dev_info['maxInputChannels'] > 0:  # Only show input devices
            print(f"Device {i}: {dev_info['name']}")
            input_devices.append((i, dev_info['name']))
    
    # Let user select a device
    while True:
        try:
            selection = input("\nEnter the device number to use: ")
            device_index = int(selection)
            if any(device_index == idx for idx, _ in input_devices):
                return device_index
            print("Invalid device number. Please try again.")
        except ValueError:
            print("Please enter a valid number.")

def get_mic_config():
    config_path = Path(__file__).parent.parent / "config" / "mic.toml"
    if config_path.exists():
        with open(config_path, "rb") as f:
            try:
                config = tomli.load(f)
                return config.get("default_audio_input", {})
            except:
                return {}
    return {}

def save_mic_config(device_name):
    config_path = Path(__file__).parent.parent / "config" / "mic.toml"
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config = {
        "default_audio_input": {
            "device_name": device_name
        }
    }
    with open(config_path, "wb") as f:
        # Add header comment
        comment = "# Configuration for default audio input device\n# This file is auto-generated when selecting a microphone device\n"
        f.write(comment.encode('utf-8'))
        tomli_w.dump(config, f)

def find_device_by_name(p, target_name):
    for i in range(p.get_device_count()):
        dev_info = p.get_device_info_by_index(i)
        if dev_info['maxInputChannels'] > 0 and dev_info['name'] == target_name:
            return i
    return None

def test_microphone():
    # Initialize PyAudio with error suppression
    with noalsaerr():
        p = pyaudio.PyAudio()
    
    # Get configuration or select new device
    config = get_mic_config()
    device_name = config.get("device_name")
    
    # Try to find device by name first
    device_index = None
    if device_name:
        device_index = find_device_by_name(p, device_name)
        if device_index is None:
            print(f"Previously configured microphone '{device_name}' not found.")
    
    # If no valid device configured or found, ask user to select one
    if device_index is None:
        device_index = select_microphone(p)
        dev_info = p.get_device_info_by_index(device_index)
        device_name = dev_info['name']
        save_mic_config(device_name)
    
    print(f"\nUsing input device: {p.get_device_info_by_index(device_index)['name']}")

    # Adjust recording parameters
    CHUNK = 4096  # Increased from 1024 to reduce chance of overflow
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100
    RECORD_SECONDS = 5
    
    try:
        # Open audio stream with selected device
        stream = p.open(format=FORMAT,
                       channels=CHANNELS,
                       rate=RATE,
                       input=True,
                       input_device_index=device_index,
                       frames_per_buffer=CHUNK)
        
        print("Recording 5 seconds of audio...")
        
        # Record audio with error handling
        frames = []
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            try:
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
            except IOError as e:
                print(f"Warning: {e}")
                continue
            
        print("Finished recording")
        
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        
        # Save the recorded data as a WAV file
        wf = wave.open("test_microphone.wav", 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        print("Saved audio to test_microphone.wav")
        return True
        
    except Exception as e:
        print(f"Error recording audio: {e}")
        return False
    finally:
        # Terminate PyAudio
        p.terminate()

if __name__ == "__main__":
    success = test_microphone()
    sys.exit(0 if success else 1)
