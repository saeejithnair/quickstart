import sounddevice as sd
import soundfile as sf
from scipy import signal
import tyro
from dataclasses import dataclass

@dataclass
class Args:
    mp3_file: str = "test_audio.mp3"
    device_name: str = "UACDemoV1.0"
    volume: float = 0.5

def play_audio(args: Args) -> None:
    try:
        # Set audio device
        device_info = sd.query_devices(args.device_name, 'output')
        device_id = device_info['index']
        device_sample_rate = device_info['default_samplerate']

        # Load and process audio
        data, sample_rate = sf.read(args.mp3_file, dtype='float32')

        # Resample if needed
        if sample_rate != device_sample_rate:
            number_of_samples = int(round(len(data) * float(device_sample_rate) / sample_rate))
            data = signal.resample(data, number_of_samples)
            sample_rate = device_sample_rate

        # Play audio
        data = data * args.volume
        sd.play(data, samplerate=sample_rate, device=device_id, blocking=True)

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    args = tyro.cli(Args)
    play_audio(args)