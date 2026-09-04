import speaker_audio_imp
import time

print("Audio test starting...")

speaker_audio_imp.play_audio("cool_robo_beat.wav")
time.sleep(4)

speaker_audio_imp.play_audio("cool_robo_beat.ogg")
time.sleep(4)

speaker_audio_imp.play_audio("cool_robo_beat.wav")
time.sleep(4)

print("Audio test finished")
