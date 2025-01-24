from elevenlabs.client import ElevenLabs
import os
from io import BytesIO
import sounddevice as sd
import soundfile as sf
import numpy as np
from scipy import signal
import dotenv

# Flag to save audio to mp3
SAVE_TO_MP3 = True

dotenv.load_dotenv()

api_key = os.getenv("ELEVENLABS_API_KEY")

client = ElevenLabs(
    api_key=api_key,
)

audio = client.generate(
    text="Each team will get one of us! A Bracket Bot!! Hackers will win challenges to take us home as the prize!!!",
    # voice="Brian",
    # voice="tdr2UZjCfSq8qpr5CfgU",
    voice="noq0H9VP5t5ouVxbKuDt", #raphael
    model="eleven_multilingual_v2"
)

# Save to mp3 if flag is set
if SAVE_TO_MP3:
    with open('output.mp3', 'wb') as f:
        for chunk in audio:
            f.write(chunk)
    print("Audio saved to output.mp3")

# Set the desired audio device
device_name = "UACDemoV1.0"  # Name of the USB audio device
device_info = sd.query_devices(device_name, 'output')
device_id = device_info['index']
device_sample_rate = device_info['default_samplerate']

# Prepare the audio data
audio_data = b''.join(audio)
data, sample_rate = sf.read(BytesIO(audio_data), dtype='float32')

# Resample if necessary
if sample_rate != device_sample_rate:
    print(f"Resampling from {sample_rate} to {device_sample_rate}")
    number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
    data = signal.resample(data, number_of_samples)
    sample_rate = device_sample_rate

# Increase the volume (optional)
volume_increase = 1.0
data = data * volume_increase

try:
    # Play the audio using the specified device
    sd.play(data, samplerate=sample_rate, device=device_id)
    sd.wait()
    print("Audio played successfully")
except sd.PortAudioError as e:
    print(f"Error playing audio: {e}")
    print(f"Supported sample rates for this device: {device_sample_rate}")