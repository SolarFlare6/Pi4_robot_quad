# imports
import sys
import select
import time
import reduced_walking

# try to import linux terminal modules for single-key reading on the raspberry pi
try:
    import tty
    import termios
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False
    try:
        import msvcrt
    except ImportError:
        msvcrt = None

# control variables
step_mode = False # False = pressing w/a/s/d walks continuously until space/x is pressed, True = runs 1 cycle per press
amplitude_step = 2.0 # degrees to change swing_amplitude when pressing + or -
bias_step = 2.0 # degrees to change swing_bias when pressing [ or ]
trim_step = 0.5 # degrees to change steer_trim when pressing , or .


# fn to read a single key press without blocking the 50 Hz servo loop
def get_key_nonblocking():
    if HAS_TERMIOS:
        dr, _, _ = select.select([sys.stdin], [], [], 0.0)
        if dr:
            ch = sys.stdin.read(1)
            # drain any buffered key-repeat characters so the loop stays responsive
            while select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.read(1)
            return ch.lower()
        return None
    elif msvcrt is not None and msvcrt.kbhit():
        ch = msvcrt.getwch()
        while msvcrt.kbhit():
            msvcrt.getwch()
        return ch.lower()
    return None

# fn to print the keyboard controls menu
def print_controls():
    mode_str = "Option A (strictly thighs = True)" if reduced_walking.strictly_thighs else "Option B (8-servo sin/cos oval CPG)"
    walk_style = "Single-cycle step" if step_mode else "Continuous (press SPACE or X to stop)"
    knees_str = "Energized (held at 175/5)" if reduced_walking.knees_energized else "Unpowered"
    print("\n========================================================")
    print("  REDUCED WALKING - WASD KEYBOARD CONTROL")
    print("========================================================")
    print("  w : Walk forward")
    print("  s : Walk backward")
    print("  a : Turn left (uses all 4 legs)")
    print("  d : Turn right (uses all 4 legs)")
    print("  SPACE or x : Stop walking and hold reduced stand")
    print("  y : Cycle turning style (cpg_trot <-> cpg_crawl <-> all_four_twist)")
    print("  , / . : Trim steering left / right (steer_trim: " + str(reduced_walking.steer_trim) + " deg)")
    print("  m : Toggle Option A (thighs) <-> Option B (8-servo sin/cos oval)")
    print("  g : Toggle gait pattern (crawl <-> trot, current: " + reduced_walking.gait_pattern + ")")
    print("  [ / ] : Decrease / increase swing_bias (current: " + str(reduced_walking.swing_bias) + " deg)")
    print("  + / - : Increase / decrease swing_amplitude (current: " + str(reduced_walking.swing_amplitude) + " deg)")
    print("  r : Flip wave time direction (wave_time_sign = " + str(reduced_walking.wave_time_sign) + ")")
    print("  f : Flip THIGH_FORWARD_SIGN (" + str(reduced_walking.THIGH_FORWARD_SIGN[3]) + "/" + str(reduced_walking.THIGH_FORWARD_SIGN[6]) + ")")
    print("  i : Flip Option B shoulder lift signs (SHOULDER_LIFT_SIGN)")
    print("  k : Toggle knees unpowered <-> energized")
    print("  c : Toggle Continuous mode <-> Single-step mode")
    print("  t : Run shoulder lift test (for Option B)")
    print("  q : Rest down and quit")
    print("--------------------------------------------------------")
    print("  Current clearance mode : " + mode_str)
    print("  Current turn_style     : " + reduced_walking.turn_style)
    print("  Current gait pattern   : " + reduced_walking.gait_pattern)
    print("  Current swing_bias     : " + str(reduced_walking.swing_bias) + " deg")
    print("  Current swing_amplitude: " + str(reduced_walking.swing_amplitude) + " deg")
    print("  Current steer_trim     : " + str(reduced_walking.steer_trim) + " deg")
    print("  Current knees state    : " + knees_str)
    print("  Current walk style     : " + walk_style)
    print("========================================================\n")

# fn to run the interactive WASD loop
def run_wasd_loop():
    global step_mode
    active_motion = None # None, "forward", "backward", "left", "right"
    motion_start_time = 0.0

    print_controls()

    old_settings = None
    if HAS_TERMIOS:
        old_settings = termios.tcgetattr(sys.stdin)

    try:
        if HAS_TERMIOS:
            tty.setcbreak(sys.stdin.fileno())

        while True:
            key = get_key_nonblocking()

            if key is not None:
                if key == "q":
                    print("\nQuitting WASD control...")
                    break

                elif key == " " or key == "x":
                    if active_motion is not None:
                        print("Stopping -> returning to reduced stand.")
                        active_motion = None
                        reduced_walking.return_to_stand()
                    else:
                        print("Already standing.")

                elif key == "y":
                    if active_motion is not None:
                        active_motion = None
                        reduced_walking.return_to_stand()
                    reduced_walking.cycle_turn_style()
                    print_controls()

                elif key == ",":
                    reduced_walking.steer_trim = round(max(-8.0, reduced_walking.steer_trim - trim_step), 1)
                    print("steer_trim decreased to " + str(reduced_walking.steer_trim) + " deg (steers more left)")

                elif key == ".":
                    reduced_walking.steer_trim = round(min(8.0, reduced_walking.steer_trim + trim_step), 1)
                    print("steer_trim increased to " + str(reduced_walking.steer_trim) + " deg (steers more right / counters left drift)")

                elif key == "m":
                    if active_motion is not None:
                        active_motion = None
                        reduced_walking.return_to_stand()
                    reduced_walking.set_strictly_thighs(not reduced_walking.strictly_thighs)
                    print_controls()

                elif key == "g":
                    reduced_walking.toggle_gait_pattern()
                    print_controls()

                elif key == "[":
                    reduced_walking.swing_bias = max(-25.0, reduced_walking.swing_bias - bias_step)
                    print("swing_bias decreased to " + str(reduced_walking.swing_bias) + " deg")
                    reduced_walking.check_swing_limits()

                elif key == "]":
                    reduced_walking.swing_bias = min(30.0, reduced_walking.swing_bias + bias_step)
                    print("swing_bias increased to " + str(reduced_walking.swing_bias) + " deg")
                    reduced_walking.check_swing_limits()

                elif key == "r":
                    reduced_walking.invert_wave_time_sign()
                    print_controls()

                elif key == "f":
                    reduced_walking.invert_thigh_forward_signs()
                    print_controls()

                elif key == "i":
                    reduced_walking.invert_shoulder_lift_signs()
                    print_controls()

                elif key == "k":
                    if active_motion is not None:
                        active_motion = None
                        reduced_walking.return_to_stand()
                    reduced_walking.set_knees_energized(not reduced_walking.knees_energized)
                    print_controls()

                elif key == "c":
                    if active_motion is not None:
                        active_motion = None
                        reduced_walking.return_to_stand()
                    step_mode = not step_mode
                    print_controls()

                elif key == "+" or key == "=":
                    reduced_walking.swing_amplitude = min(30.0, reduced_walking.swing_amplitude + amplitude_step)
                    reduced_walking.turn_amplitude = min(30.0, reduced_walking.turn_amplitude + amplitude_step)
                    print("swing_amplitude=" + str(reduced_walking.swing_amplitude) + " deg, turn_amplitude=" + str(reduced_walking.turn_amplitude) + " deg")
                    reduced_walking.check_swing_limits()

                elif key == "-" or key == "_":
                    reduced_walking.swing_amplitude = max(2.0, reduced_walking.swing_amplitude - amplitude_step)
                    reduced_walking.turn_amplitude = max(2.0, reduced_walking.turn_amplitude - amplitude_step)
                    print("swing_amplitude=" + str(reduced_walking.swing_amplitude) + " deg, turn_amplitude=" + str(reduced_walking.turn_amplitude) + " deg")

                elif key == "t":
                    if active_motion is not None:
                        active_motion = None
                        reduced_walking.return_to_stand()
                    reduced_walking.shoulder_lift_test()
                    print_controls()

                elif key in ("w", "s", "a", "d"):
                    if step_mode:
                        # run 1 cycle using the imported blocking functions
                        if key == "w":
                            reduced_walking.walk_forward(1)
                        elif key == "s":
                            reduced_walking.walk_backward(1)
                        elif key == "a":
                            reduced_walking.turn_left(1)
                        elif key == "d":
                            reduced_walking.turn_right(1)
                    else:
                        # continuous mode
                        new_motion = {"w": "forward", "s": "backward", "a": "left", "d": "right"}[key]
                        if active_motion != new_motion:
                            active_motion = new_motion
                            motion_start_time = time.time()
                            print("Moving : " + active_motion + " (press SPACE or X to stop)")

            # update the servos at 50 Hz if a continuous motion is active
            if active_motion is not None:
                elapsed = time.time() - motion_start_time
                ramp = reduced_walking.smooth(min(1.0, elapsed / (reduced_walking.step_time * 0.5)))

                if active_motion == "forward":
                    reduced_walking.update_gait_step(elapsed, 1, 1, ramp)
                elif active_motion == "backward":
                    reduced_walking.update_gait_step(elapsed, -1, -1, ramp)
                elif active_motion == "left":
                    reduced_walking.update_gait_step(elapsed, -1, 1, ramp)
                elif active_motion == "right":
                    reduced_walking.update_gait_step(elapsed, 1, -1, ramp)

            time.sleep(reduced_walking.loop_dt)

    finally:
        if HAS_TERMIOS and old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# fn to run the WASD test script
def main():
    try:
        print("WASD Reduced Walking Test started...")
        input("Put the robot in the rest pose, then press Enter to stand up > ")

        if not reduced_walking.setup_robot():
            return

        run_wasd_loop()

    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected - stopping script...")

    except Exception as e:
        print("\nError happened : ", e)

    finally:
        reduced_walking.shutdown_robot()
        print("Finished")

if __name__ == "__main__":
    main()
