from gpiozero import TonalBuzzer
from time import sleep

tb = TonalBuzzer(15) # Connected to GPIO 20

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