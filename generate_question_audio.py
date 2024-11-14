import os
import json
import time
import requests
from dotenv import load_dotenv
from tqdm import tqdm

# Load environment variables
load_dotenv()

def generate_audio(text, output_filename, retry_delay=60):
    if os.path.exists(output_filename):
        return True
        
    url = "https://api.play.ht/api/v2/tts/stream"
    headers = {
        'X-USER-ID': os.getenv('X_USER_ID'),
        'AUTHORIZATION': os.getenv('AUTHORIZATION'),
        'accept': 'audio/mpeg',
        'content-type': 'application/json'
    }
    
    payload = {
        "text": text,
        "voice_engine": "Play3.0",
        "voice": "s3://voice-cloning-zero-shot/3b74b785-e06e-4a53-a9c4-ebed08a70ef2/susannarrativesaad/manifest.json",
        "output_format": "mp3",
    }

    while True:
        response = requests.post(url, json=payload, headers=headers)
        
        if response.status_code == 200:
            with open(output_filename, 'wb') as f:
                f.write(response.content)
            print(f"\nGenerated: {output_filename}")
            return True
        elif "RATE_LIMIT_EXCEEDED" in response.text:
            print(f"\nRate limit hit. Waiting {retry_delay} seconds...")
            for _ in tqdm(range(retry_delay), desc="Cooling down"):
                time.sleep(1)
        else:
            print(f"\nError: {response.status_code}")
            print(response.text)
            return False

# Create directory structure
base_dir = "audio_questions"
os.makedirs(os.path.join(base_dir, "hackathon"), exist_ok=True)
os.makedirs(os.path.join(base_dir, "quirky"), exist_ok=True)

# Load questions
with open('questions.json', 'r') as f:
    questions = json.load(f)

# Process questions with progress tracking
def process_question_set(questions, category):
    for idx, question in enumerate(tqdm(questions, desc=f"Processing {category} questions")):
        filename = f"{category}/question_{idx:02d}.mp3"
        output_path = os.path.join(base_dir, filename)
        if not generate_audio(question, output_path):
            print(f"Failed to generate audio for {category} question {idx}")
            return False
    return True

# Generate audio for all questions
process_question_set(questions['hackathon_questions'], 'hackathon')
process_question_set(questions['quirky_questions'], 'quirky')

# Create mapping file
question_map = {
    "hackathon": {
        f"question_{idx:02d}.mp3": question 
        for idx, question in enumerate(questions['hackathon_questions'])
    },
    "quirky": {
        f"question_{idx:02d}.mp3": question 
        for idx, question in enumerate(questions['quirky_questions'])
    }
}

with open(os.path.join(base_dir, "question_map.json"), 'w') as f:
    json.dump(question_map, f, indent=2)