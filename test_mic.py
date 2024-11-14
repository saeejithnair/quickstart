import pyaudio
import wave
import time
import sys
import select
import numpy as np
import scipy.signal as signal

# Audio parameters
CHUNK = 2048  # Number of frames per buffer
FORMAT = pyaudio.paInt16  # Changed to Int16 for WAV compatibility
CHANNELS = 1
RATE = 44100  # Sampling rate in Hz
MAX_SECONDS = 15
OUTPUT_FILE = "test_recording.wav"

def high_pass_filter(data, cutoff=300, fs=RATE, order=5):
    b, a = signal.butter(order, cutoff / (0.5 * fs), btype='high')
    return signal.filtfilt(b, a, data)

def record_audio():
    p = pyaudio.PyAudio()
    
    # Open input stream
    stream = p.open(
        format=FORMAT,
        channels=CHANNELS,
        rate=RATE,
        input=True,
        frames_per_buffer=CHUNK
    )

    print("Starting recording... Press Enter to stop, or wait 15 seconds")
    
    # Record audio data
    frames = []
    start_time = time.time()
    
    while True:
        data = stream.read(CHUNK)
        frames.append(data)
        
        # Check if Enter was pressed
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = input()  # Read the Enter keypress
            break
                
        # Check if max time reached
        if time.time() - start_time >= MAX_SECONDS:
            break
        
    print("Recording finished!")
    
    # Stop and close the stream
    stream.stop_stream()
    stream.close()
    p.terminate()
    
    # Process audio
    print("Processing audio...")
    gain = 10.0  # Adjust as needed
    audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
    
    # Apply high-pass filter
    filtered_data = high_pass_filter(audio_data, cutoff=300, fs=RATE)
    
    # Apply gain
    amplified_data = np.clip(filtered_data * gain, -32768, 32767).astype(np.int16)
    
    # Save the recorded data as a WAV file
    wf = wave.open(OUTPUT_FILE, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(amplified_data.tobytes())
    wf.close()
    
    print(f"Audio saved to {OUTPUT_FILE}")

if __name__ == "__main__":
    record_audio()