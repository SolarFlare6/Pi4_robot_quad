# import os to set audio output
import os

# set the output device
os.environ["SDL_AUDIODRIVER"] = "alsa"
os.environ["AUDIODEV"] = "plughw:2,0"

# import the rest of the libraries
import pygame
import time
import threading

_initialized = False

def _init():
    global _initialized
    if not _initialized:
        pygame.mixer.init(
            frequency=44100,
            size=-16,
            channels=2,
            buffer=4096
        )
        _initialized = True

def _play(file_path):
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()

def play_audio(file_path):
    _init()

    if not os.path.exists(file_path):
        print("File not found:", file_path)
        return

    print("Playing:", file_path)

    threading.Thread(
        target=_play,
        args=(file_path,),
        daemon=True
    ).start()
