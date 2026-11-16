import os

# set the output device (must be set before pygame is imported)
os.environ["SDL_AUDIODRIVER"] = "alsa"
os.environ["AUDIODEV"] = "plughw:2,0"

import io
import re
import wave
import time
import queue
import atexit
import threading

import pygame
from piper import PiperVoice, SynthesisConfig

# voice setting
# define the model to be used in the line below
VOICE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "voices", "jarvis-medium.onnx"
)

# old voice model setting
"""VOICE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "voices", "en_US-lessac-medium.onnx"
)"""

# length_scale: higher = slower speech (1.0 is normal)
SPEECH_CONFIG = SynthesisConfig(
    volume=1.7,
    length_scale=1.0,
    noise_scale=0.667,
    noise_w_scale=0.8,
)
# ------------------------------------------

_initialized = False
_voice = None
_init_lock = threading.Lock()
_voice_lock = threading.Lock()

_speech_queue = queue.Queue()            # text waiting to be synthesized
_play_queue = queue.Queue(maxsize=2)     # synthesized sounds waiting to be played
_synth_thread = None
_player_thread = None
_thread_lock = threading.Lock()

_pending = 0                             # number of speak requests not finished yet
_pending_cond = threading.Condition()


def _init():
    """Start the audio mixer once."""
    global _initialized
    with _init_lock:
        if not _initialized:
            pygame.mixer.init(
                frequency=44100,
                size=-16,
                channels=2,
                buffer=4096,
            )
            _initialized = True


def _get_voice():
    """Load the Piper voice once (first call is slow, later calls are fast)."""
    global _voice
    with _voice_lock:
        if _voice is None:
            _voice = PiperVoice.load(VOICE_PATH)
        return _voice


def _synthesize(text):
    """Turn text into an in-memory WAV and return it as a pygame Sound."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wav_file:
        _get_voice().synthesize_wav(text, wav_file, syn_config=SPEECH_CONFIG)
    buf.seek(0)
    return pygame.mixer.Sound(file=buf)


def _split_sentences(text):
    parts = re.split(r"(?<=[.!?])\s+", text.strip())
    return [p for p in parts if p]


def _synth_worker():
    """Turns queued text into sounds, one sentence at a time."""
    while True:
        text = _speech_queue.get()
        try:
            _init()
            for sentence in _split_sentences(text):
                _play_queue.put(_synthesize(sentence))
        except Exception as e:
            print("Speech error:", e)
        finally:
            _play_queue.put(None)        # marks the end of this request
            _speech_queue.task_done()


def _player_worker():
    """Plays sounds back to back while the next sentence is being synthesized."""
    global _pending
    while True:
        sound = _play_queue.get()
        if sound is None:
            with _pending_cond:
                _pending -= 1
                _pending_cond.notify_all()
            continue
        channel = sound.play()
        while channel is not None and channel.get_busy():
            time.sleep(0.05)


def _ensure_speech_threads():
    global _synth_thread, _player_thread
    with _thread_lock:
        if _synth_thread is None:
            _synth_thread = threading.Thread(target=_synth_worker, daemon=True)
            _synth_thread.start()
            _player_thread = threading.Thread(target=_player_worker, daemon=True)
            _player_thread.start()


def robot_speak(text, wait=False):
    """
    Speak a string. Returns immediately unless wait=True.
    Multiple calls are queued and spoken in order.
    """
    global _pending
    
    print("Running voice synth with following string : " + text)
    
    if not text or not text.strip():
        return
    _ensure_speech_threads()
    with _pending_cond:
        _pending += 1
    _speech_queue.put(text)
    if wait:
        wait_for_speech()


def robot_speak_with_volume(txt,vol=1.0):
    
    global SPEECH_CONFIG
    print("Running robot speaker with volume adjustments with params : " + " txt : " + str(txt) + ", volume : " + str(vol))
    
    SPEECH_CONFIG = SynthesisConfig(
        volume=vol,
        length_scale=1.0,
        noise_scale=0.667,
        noise_w_scale=0.8,
    )
    
    robot_speak(str(txt))


def set_volume(vol=1.0):
    """Adjusts default synthesis and mixer playback volume."""
    global SPEECH_CONFIG
    vol_float = float(vol)
    SPEECH_CONFIG = SynthesisConfig(
        volume=vol_float,
        length_scale=1.0,
        noise_scale=0.667,
        noise_w_scale=0.8,
    )
    try:
        if _initialized and pygame.mixer.get_init():
            pygame.mixer.music.set_volume(max(0.0, min(1.0, vol_float)))
    except Exception:
        pass


def stop_audio():
    """Immediately stops active speech and audio file playback."""
    try:
        while not _speech_queue.empty():
            _speech_queue.get_nowait()
            _speech_queue.task_done()
    except Exception:
        pass
    try:
        if _initialized and pygame.mixer.get_init():
            pygame.mixer.stop()
            pygame.mixer.music.stop()
    except Exception:
        pass


def wait_for_speech():
    """Block until everything queued has finished speaking."""
    with _pending_cond:
        while _pending > 0:
            _pending_cond.wait(timeout=0.1)


def preload_voice():
    """Optional: call at startup so the first sentence has no delay."""
    _init()
    _get_voice()


def _play(file_path):
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()


def play_audio(file_path):
    """Play an audio file (non-blocking), same as the original."""
    _init()

    if not os.path.exists(file_path):
        print("File not found:", file_path)
        return

    print("Playing:", file_path)

    threading.Thread(target=_play, args=(file_path,), daemon=True).start()


# Make sure the script never exits mid-sentence
atexit.register(wait_for_speech)
