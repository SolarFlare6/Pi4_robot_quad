# robot_pi/voice/voice_synth.py
"""AI Voice Synthesizer & Speaker Interface module for the Quadruped Robot.

Bridges `speak()`, `stop_speaking()`, `play_audio()`, `set_volume()`, and `preload_voice()`
directly to the Piper TTS & Pygame ALSA speaker driver in `drivers/upgraded_audio_interface_imp.py`
(using the ONNX voice models in `drivers/voices/`).
"""

import logging

logger = logging.getLogger("robot.voice")


def speak(text: str, volume: float = 1.7, wait: bool = False) -> bool:
    """Makes the robot speak a given string of text at the specified volume using Piper TTS.

    Args:
        text (str): The sentence or prompt for the robot to speak.
        volume (float): Piper synthesis volume level (e.g. 0.0 to 2.0+, default 1.7).
        wait (bool): If True, blocks until speech finishes playing.

    Returns:
        bool: True if speech was successfully queued/played.
    """
    logger.info("speak() requested: text='%s' (volume=%.2f)", text, volume)
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.robot_speak_with_volume(str(text), vol=float(volume))
        if wait:
            upgraded_audio_interface_imp.wait_for_speech()
        return True
    except Exception as exc:
        logger.error("Voice synth error in upgraded_audio_interface_imp: %s", exc)
        return False


def stop_speaking() -> None:
    """Immediately stops any currently active voice or speaker audio playback."""
    logger.info("stop_speaking() called.")
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.stop_audio()
    except Exception as exc:
        logger.debug("stop_speaking fallback error: %s", exc)


def play_audio(file_path: str) -> bool:
    """Plays an audio file non-blocking through the USB speaker."""
    logger.info("play_audio() requested: file_path='%s'", file_path)
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.play_audio(file_path)
        return True
    except Exception as exc:
        logger.error("play_audio error: %s", exc)
        return False


def set_volume(volume: float = 1.7) -> bool:
    """Adjusts the speaker and Piper TTS synthesis volume."""
    logger.info("set_volume() requested: volume=%.2f", volume)
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.set_volume(float(volume))
        return True
    except Exception as exc:
        logger.error("set_volume error: %s", exc)
        return False


def preload_voice() -> bool:
    """Preloads the Piper ONNX voice model into memory so the first spoken sentence has zero delay."""
    logger.info("Preloading Piper ONNX voice model...")
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.preload_voice()
        return True
    except Exception as exc:
        logger.error("preload_voice error: %s", exc)
        return False
