import os
from pathlib import Path

import openai

openai.api_key = os.getenv("OPENAI_API_KEY")

with open(Path.home() / "Downloads/audiofile.mp3", "rb") as af:
  transcript = openai.Audio.transcribe("whisper-1", af)
  print(transcript)
