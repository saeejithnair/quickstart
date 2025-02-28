#!/usr/bin/env python3
"""
Test script for the Kokoro Text-to-Speech package.

pip install this package https://github.com/hexgrad/kokoro

i had to do this to install spacy "pip install --only-binary :all: spacy"
"""

# Import the necessary modules from kokoro
from kokoro.pipeline import KPipeline
import numpy as np
from scipy.io.wavfile import write
import time

# Define a sample text to convert to speech
sample_text = "Hello World, im Bracket Bot, and I generated this audio entirely on my own."


def main():
    # Initialize the Kokoro pipeline for American English
    # This loads the model and all necessary components
    print("Initializing Kokoro pipeline...")
    pipeline = KPipeline(lang_code='a')
    
    # Generate speech from the sample text
    print(f"Generating speech for: '{sample_text}'")

    # Start timing the audio generation
    start_time = time.time()

    # Using the generator pattern as shown in the README
    audio_chunks = []
    for gs, ps, audio in pipeline(sample_text, voice='af_heart'):
        # gs: graphemes (text), ps: phonemes, audio: audio data
        # Only append the audio part to our collection
        # Convert PyTorch tensor to NumPy array
        audio_chunks.append(audio.detach().cpu().numpy())

    # Get the final audio by concatenating all chunks
    audio = np.concatenate(audio_chunks) if len(audio_chunks) > 1 else audio_chunks[0]
    
    # If the result is still a tensor, convert it to NumPy
    if hasattr(audio, 'detach'):
        audio = audio.detach().cpu().numpy()

    # Stop timing after audio generation is complete
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Audio generation took {elapsed_time:.2f} seconds.")

    # Convert to 16-bit PCM (the standard for WAV files)
    audio_int16 = (audio * 32767).astype(np.int16)

    # Save the audio to a WAV file
    # The sampling rate is 24000 Hz (as used by Kokoro)
    output_file = "output.wav"
    print(f"Saving audio to {output_file}")
    write(output_file, 24000, audio_int16)
    print("Done! Audio generated successfully.")

if __name__ == "__main__":
    main()
