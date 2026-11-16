from gpiozero import TonalBuzzer
from time import sleep

tb = TonalBuzzer(23) # Connected to GPIO 20

# play a given midi note
def play_tone(tone):
    print("Playing tone...")
    tb.play(tone) # MIDI note 60
    sleep(1)
    tb.stop()


# fn play list of notes example (['C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4', 'C5']) <- midi notes list
def play_list_of_notes(list):
    print("Playing given list of notes...")
    for note in list:
        tb.play(note)
    sleep(0.3)
    print("Finished playing")
    tb.stop()


# play with frequency and duration
def play_tone_adv(f,d):
    print("Playing tone with frequency,duration : " + str(f) + "," + str(d))
    try:
        tb.play(float(f))
    except ValueError:
        # Clamp to TonalBuzzer's supported frequency range (default 220Hz - 880Hz) if out of bounds
        clamped_f = max(tb.min_tone.frequency, min(tb.max_tone.frequency, float(f)))
        tb.play(clamped_f)
    sleep(d)
    tb.stop()

