import os
import random
import json
import sounddevice as sd
import soundfile as sf
from scipy import signal
from dataclasses import dataclass
from datetime import datetime
from pydub import AudioSegment
import select  # Added import
import numpy as np  # Added import

@dataclass
class SpeakerArgs:
    device_name: str = "UACDemoV1.0"
    volume: float = 0.5

def play_audio(file_path: str, args: SpeakerArgs) -> None:
    try:
        device_info = sd.query_devices(args.device_name, 'output')
        device_id = device_info['index']
        device_sample_rate = device_info['default_samplerate']

        data, sample_rate = sf.read(file_path, dtype='float32')

        if sample_rate != device_sample_rate:
            number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
            data = signal.resample(data, number_of_samples)
            sample_rate = device_sample_rate

        data = data * args.volume
        sd.play(data, samplerate=sample_rate, device=device_id, blocking=True)

    except Exception as e:
        print(f"Error playing audio: {e}")

def high_pass_filter(data, cutoff=300, fs=44100, order=5):
    b, a = signal.butter(order, cutoff / (0.5 * fs), btype='high')
    return signal.filtfilt(b, a, data)

def record_audio(output_path: str, max_seconds: int = 15, gain: float = 15.0) -> None:
    CHUNK = 2048
    FORMAT = 'int16'
    CHANNELS = 1
    RATE = 44100

    try:
        print("Recording... Press Enter to stop.")
        recording = []
        start_time = datetime.now()

        with sd.InputStream(samplerate=RATE, channels=CHANNELS, dtype=FORMAT) as stream:
            while True:
                if (datetime.now() - start_time).seconds >= max_seconds:
                    break
                data, _ = stream.read(CHUNK)
                recording.append(data)
                if os.sys.stdin in select.select([os.sys.stdin], [], [], 0)[0]:
                    input()
                    break

        print("Processing audio...")
        audio_data = np.frombuffer(b''.join(recording), dtype=np.int16)
        filtered_data = high_pass_filter(audio_data)
        amplified_data = np.clip(filtered_data * gain, -32768, 32767).astype(np.int16)

        sf.write(output_path, amplified_data, RATE)
        print(f"Audio recorded to {output_path}")

    except Exception as e:
        print(f"Error recording audio: {e}")

def combine_audios(audio_files: list, output_file: str) -> None:
    combined = AudioSegment.empty()
    for file in audio_files:
        if file.endswith('.mp3'):
            segment = AudioSegment.from_mp3(file)
        elif file.endswith('.wav'):
            segment = AudioSegment.from_wav(file)
        combined += segment
    combined.export(output_file, format="wav")
    print(f"Interview saved to {output_file}")

def run_interview():
    with open('questions.json', 'r') as f:
        questions = json.load(f)

    hack_questions = questions['hackathon_questions']
    quirky_questions = questions['quirky_questions']

    selected_hack = random.choice(list(enumerate(hack_questions)))
    selected_quirky = random.choice(list(enumerate(quirky_questions)))

    hack_index, hack_question = selected_hack
    quirky_index, quirky_question = selected_quirky

    hack_mp3 = f"audio_questions/hackathon/question_{hack_index:02d}.mp3"
    quirky_mp3 = f"audio_questions/quirky/question_{quirky_index:02d}.mp3"

    speaker_args = SpeakerArgs()

    timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
    response1 = f"responses/hack_response_{timestamp}.wav"
    response2 = f"responses/quirky_response_{timestamp}.wav"

    os.makedirs('responses', exist_ok=True)

    # Ask Hackathon Question
    print(f"Question: {hack_question}")
    play_audio(hack_mp3, speaker_args)
    record_audio(response1)

    # Ask Quirky Question
    print(f"Question: {quirky_question}")
    play_audio(quirky_mp3, speaker_args)
    record_audio(response2)

    # Combine Audios
    interview_file = f"interviews/interview_{timestamp}.wav"
    os.makedirs('interviews', exist_ok=True)
    combine_audios([hack_mp3, response1, quirky_mp3, response2], interview_file)

if __name__ == "__main__":
    run_interview()