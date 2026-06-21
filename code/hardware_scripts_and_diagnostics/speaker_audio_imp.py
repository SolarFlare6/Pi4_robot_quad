# import os to set audio output
import os

# set the output device
os.environ["SDL_AUDIODRIVER"] = "alsa"
os.environ["AUDIODEV"] = "plughw:2,0"

# import the rest of the libraries
import pygame
import time

# start pygame mixer
pygame.mixer.init()

# audio play fn
def play_audio(file_path):
    print("Running play_audio fn with param : " + str(file_path))
    if not os.path.exists(file_path):
        print("File not found:", file_path)
        return

    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy():
        time.sleep(0.05)