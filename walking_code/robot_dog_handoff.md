# Robot Dog: handoff document for a new agent (zero context)

Status date: 2026-11-07. Written by the previous assistant so a new agent can continue the project without any other history. Read sections 0 to 4 first.

Legend: **(user)** = stated by the user. **(photo)** = read from the user's photos. **(derived)** = calculated. **(unverified)** = an assumption that must be checked on the real robot before relying on it.

---

## 0. Summary

A hobbyist is building a quadruped robot dog on a Raspberry Pi 4. The body and legs are 3D printed from the open project `michaelkubina/SpotMicroESP32`, but the user mounted the legs differently from the original, so servo zero positions and directions are NOT the repo's.

- **Works:** all hardware, standing in a hand-tuned pose, continuous gyro leveling while standing, LEDs, buzzer, speaker audio.
- **Does not work:** walking. Every gait tried either fell over or did not move (section 7).
- **Root causes (derived):** (1) the old inverse kinematics was fitted to a single pose and omitted the 60.5 mm hip offset, (2) balance was applied through the shoulder servos, which can only swing legs sideways, (3) the "shuffle" and "creep" gaits never lifted the swinging foot so they dragged the body back, (4) the trot never shifted weight over the remaining feet, (5) the robot is top heavy (Pi 4, speakers and wiring on top).
- **New foundation:** `kinematics_core.py` (Appendix A) is a validated pure-math leg and body-pose IK built from the repo's real dimensions. It is not connected to servos yet.
- **Next step:** build a calibration tool that maps geometric joint angles to servo angles, then a single-writer control loop, then a crawl gait, then `reduced_mode` (sections 9 and 10).

---

## 1. What the user wants

1. Stand stably with automatic gyro correction. (Done in a basic form.)
2. Walk with keyboard control: `w` forward, `s` backward, `a` turn left, `d` turn right. (Not working.)
3. A variable `reduced_mode`. When `True` the robot walks using only the middle-leg motors (the thigh motors, indexes 3, 6, 9, 12). The lower-leg motors (4, 7, 10, 13) stay in their collapsed (resting) state and are never extended. See section 10. (Not built.)
4. A simulator to tune walking before running it on hardware, ideally inside the user's Flutter app. (A first browser simulator and Flutter widget exist but use the OLD approximate kinematics, see section 6.)
5. Optional: natural-language control with the Needle 2 model (`cactus-compute/needle`, a ~14 MB tool-calling model). Not implemented. An early draft guessed at its API and was dropped. Verify the real API before using it.

---

## 2. Hardware

| Part | Detail |
|---|---|
| Controller | Raspberry Pi 4. Scripts are run with `sudo python3`. User's scripts live in `/home/pi/Desktop/machine_make/hardware_header` (user) |
| Servo driver | PCA9685, 16 channels, I2C. SDA = GPIO2 (pin 3), SCL = GPIO3 (pin 5). Logic from Pi 3.3 V. Servo V+ from a separate supply, grounds shared (user) |
| Servos | 12x DS3218MG for the legs, plus 2 camera pan/tilt servos at indexes 0 (x) and 1 (y) (user) |
| IMU | GY-521 (MPU6050), I2C address 0x68 (user) |
| LED strip | 8 WS2812 LEDs, data on GPIO10 (SPI0 MOSI). Moved from PWM GPIO18 to SPI because the Pi's headphone-jack audio uses the same PWM/DMA hardware; mixing them produced static until reboot. Fixed by the move (user) |
| Buzzer | GPIO23 (user) |
| Audio | Pi headphone jack -> audio amplifier module (5 V) -> two speakers. `pygame` with `SDL_AUDIODRIVER=alsa`, `AUDIODEV=plughw:2,0` (user) |
| Camera | Pan/tilt head on the front, servos 0 and 1 (user) |
| Cooler | Fan on the Pi 5 V / GND header pins (user) |
| Pi power | USB-C PD charger, 5 V (user) |
| Servo power | Planned: 12 V 10 A adapter -> XL4016 buck module -> about 6.5 V -> servo rail. See section 8 |

SPI LED mode on a Pi 4 needs SPI enabled and (recommended by the rpi_ws281x docs) a fixed core clock in `/boot/firmware/config.txt`: `core_freq=500` and `core_freq_min=500` (unverified whether the user added these).

Photos (user): the robot has a lot of loose wiring on top. The Pi 4 and a perfboard carrying the gyro and screw terminals sit on 3D printed plates above the body. The robot is top heavy.

---

## 3. Software environment and the user's existing modules

All of these are the user's own files, working, and should be imported, not rewritten. Ask the user to attach the real files.

- **`servo_driver_imp.py`** uses `adafruit_servokit` (`kit = ServoKit(channels=16)`).
  - `set_servo_range(servo_index, max_range)` sets pulse width range 500 to 2500 us and `actuation_range`. The user always uses 180.
  - `driver_set_servo_angle_with_index(index, angle)` casts to `int` and prints on every call.
  - `snap_servo_left/right(servo_index)`, `pan_to_left/right(current_rot, servo_index, pan_speed)`.
  - `cleanup_servos()` sets `angle = None` on all 16 channels, which RELEASES the servos.
  - Known weaknesses: integer truncation (a few mm at the foot), prints in every call (slows loops), one I2C write per call, releases torque on cleanup.
- **`gyro_imp.py`** uses `from mpu6050 import mpu6050; sensor = mpu6050(0x68)`.
  - `get_sensor_accel()`, `get_sensor_gyro()`, `get_pitch_roll()` returning `{"pitch": deg, "roll": deg}`.
  - Pitch and roll come from the accelerometer only (`atan2`), no gyro integration, prints on every call, noisy (about +/-3 degrees while the robot stands still).
  - The mapping between the sensor axes and the robot's axes has NOT been verified (unverified). Check sign by tilting the nose down.
- **`rgb_led_imp.py`** uses `rpi_ws281x` (`PixelStrip(8, 10, 800000, 10, False, 64, 0)`, `strip.begin()` runs at import). Functions: `turn_on_strip_with_color(r,g,b)`, `turn_off_strip()`, `turn_on_led_at_index(...)`, `blink_oke()` (green, 0.5 s, blocking), `blink_warning()`, `flash_alert()`, `animate_running_process()`, `move_pixel_from_left_to_right(...)`.
- **`speaker_audio_imp.py`** must be imported FIRST in any script, followed by `time.sleep(1)`, before other hardware imports. `play_audio(file)` plays in a background thread.
- **`mini_speaker_imp.py`** drives the buzzer: `play_tone(70)`, `play_list_of_notes([...])`.
- The user also has a manual servo jog script that asks for servo indexes, angle and range, then calls `set_servo_range` and `driver_set_servo_angle_with_index`.

---

## 4. Servo map and known-good angles

Indexes (user). Each leg has three motors: 1 = shoulder (swings the leg sideways), 2 = upper/middle leg (the thigh), 3 = lower leg (the knee). The user's code calls the thigh motors the "main" or "middle" legs.

| Index | Name | Joint |
|---|---|---|
| 0, 1 | CAM_X, CAM_Y | camera pan, tilt |
| 2, 3, 4 | FL_MOTOR_1, _2, _3 | front left shoulder, thigh, knee |
| 5, 6, 7 | FR_MOTOR_1, _2, _3 | front right |
| 8, 9, 10 | BL_MOTOR_1, _2, _3 | back left |
| 11, 12, 13 | BR_MOTOR_1, _2, _3 | back right |

All legs use `actuation_range = 180`. From the user's notebook: left lower legs (4, 10) move inverse (rest 180), right lower legs (7, 13) normal (rest 0); left thighs (3, 9) rest at 180, right thighs (6, 12) rest at 0.

**Resting angles (user, authoritative):**

```
2:110  3:180  4:180
5:110  6:0    7:0
8:130  9:180  10:180
11:110 12:0   13:0
```

**Standing set 1 (hand-tuned, worked, first standing photo):**

```
2:110  3:50   4:90
5:110  6:130  7:90
8:130  9:50   10:90
11:80  12:130 13:90
```

**Standing set 2 ("better standing", hand-tuned later, the one the user called good):**

```
2:110  3:70   4:50
5:110  6:110  7:150
8:130  9:70   10:40
11:90  12:90  13:160
```

Note: index 11 is 80 in set 1 and 90 in set 2. An earlier reply listed 80 for set 2 by mistake; 90 is correct for set 2.

**Rest pose as seen in the user's photos (photo, unverified for all four legs):** every thigh lies horizontal and points forward from its shoulder. Each lower leg is folded back underneath its thigh, so the round knurled foot tip ends up near the shoulder and the knee end cap is at the forward end of the folded leg. What touches the table is the flat underside of the folded lower leg plus the foot tip. The body chassis sits above the table.

---

## 5. Geometry

User's rough measurements (user): middle leg 11.5 cm, lower leg 14 cm, body 26.5 x 11 x 7 cm (outer size).

Repo IK document (user pasted it; all mm, between rotation-axis centers, foot-tip center):

```
L1: 10 mm
L2: 60.5 mm
L3: 111.1 mm   (L3a 107 mm / L3b 30 mm: the thigh is a bent part)
L4: 118.5 mm
foot tip adds ~20 mm of height and is ignored
body plane: 78 mm wide x 207.5 mm long, between shoulder axes
(207.5 = 150 chassis + 2 x 5 shoulders + 2 x 23.75 half shoulder blocks)
```

How these relate to the user's rough numbers (derived): 118.5 + 20 foot tip = 138.5 mm, which matches the user's 14 cm lower leg. 111.1 mm matches the 11.5 cm thigh. 26.5 x 11 cm is the outer body, the IK needs 207.5 x 78 mm.

Interpretation used in `kinematics_core.py` (unverified): 60.5 mm = sideways offset of the leg plane from the shoulder axis (`HIP_OFFSET`), 10 mm = offset inside the leg plane from the shoulder axis to the thigh axis (`HIP_DROP`, assumed to be BELOW the shoulder axis). The doc labels them L1 = 10 and L2 = 60.5; the assignment is inferred from their sizes.

The thigh is bent: the straight line between its two joint axes is 111.1 mm, but the physical edge of the part is about 15.7 degrees off that line (derived: atan(30/107)). Calibration must reference the joint-axis line, not the edge of the part.

Knee direction: the standing photo looks like the knees point BACKWARD, so `KNEE_DIR = -1` in the code (unverified).

---

## 6. Files and how far to trust them

| File | What it is | Trust |
|---|---|---|
| user modules in section 3 | the user's working hardware modules | trust |
| `stand_up_leveled.py` | user-style standing script. Sets shoulders, then thighs, then knees with staggered delays (to limit current), then starts a background gyro leveling thread. Flags: `better_standing` (uses set 2), `full_leveling` (also adjusts thighs). Leveling uses the shoulder servos with `GYRO_PITCH_OFFSET = 3.57`, `GYRO_ROLL_OFFSET = 5.39`, gain 1.2, deadband 1.0 deg, max correction 15 deg, interval 0.15 s, pitch gain x1.5. Shutdown folds the legs to rest and releases servos | works, the user is happy with the stand. DO NOT change its behaviour. Its leveling physics is weak (see section 7) but it works for standing |
| `kinematics_core.py` | NEW pure-math leg IK/FK and body-pose IK from the repo dimensions (Appendix A). Self-test passes: IK then FK error about 1e-13 mm, planted feet drift about 1e-14 mm under body roll, pitch, yaw, shift, height | math trusted, physical assumptions in section 5 unverified |
| `kinematics_imp.py` | OLD IK with offsets back-solved from one pose and no hip offset | DO NOT USE |
| `main_ik.py`, `walk_control.py` | OLD IK stand and walking scripts (trot, shuffle, creep) | DO NOT USE |
| `robot_simulator.html` (published as a private Claude artifact) | browser 3D simulator with sliders, WASD gait, copy-angles button | uses the OLD approximate IK, rebuild on `kinematics_core.py` |
| `robot_simulator_widget.dart` | Flutter `CustomPaint` version of the simulator | same limitation |
| handwritten PDF `Pi4_robot_dogo.pdf` | user's pin map, wiring sketches, servo mapping, notebook of neutral/start/final angles | reference |

---

## 7. Walking history and lessons (important)

1. **Trot, feet lifted 3 cm, diagonal pairs, 3 cm steps:** fell over immediately. No weight shift onto the remaining feet, top-heavy robot, jerky motion.
2. **Shuffle gait, feet never lifted, slid forward and back:** stood, then fell forward when started. Later: stable but did not move, because the swinging foot is dragged along the table and cancels the push.
3. **Creep gait, one leg at a time with gyro stability waits:** stable but did not move, same cause (feet not lifted). Diagonal pairs without lift also did not move.
4. **Dynamic shoulder-base tracking plus a servo lock:** the user said it "almost broke the robot" and asked to restore the original standing behaviour. Rule: never alter the working stand; add walking behaviour separately.
5. **Leveling through shoulders:** shoulders swing legs sideways, so they cannot fix pitch and only weakly help roll. Correct approach: body-pose IK that changes leg heights while feet stay planted.
6. **Base mismatch bug:** the old IK stood back-right shoulder at 90 but the leveling base used 80.
7. **Loop quality:** the driver prints on every call and truncates to integer degrees; this makes motion rough. Tilt is accelerometer-only and noisy.
8. **Surface and mass:** walking was on a smooth desk. Wiring was only partially tidied. TPU or rubber foot tips are recommended by the repo.
9. **Gyro data (user readings):**
   - robot flat on the table, 5 readings: pitch 2.87, 2.47, 4.03, 2.82, 5.68 (avg 3.57); roll 5.63, 6.06, 5.76, 5.89, 3.61 (avg 5.39)
   - standing in the old IK pose, 5 readings: pitch -3.94, -4.64, -2.45, 0.58, 2.73 (avg -1.55); roll -0.42, -2.21, -0.93, -0.07, 0.25 (avg -0.68)

---

## 8. Power

- Planned: 12 V 10 A adapter (barrel jack, probably 5.5/2.1 mm, center positive, unverified) -> XL4016 8 A buck module (display, "Radj" trimmer; the user has one, its display showed 12.5 V before adjustment) -> servo rail. The user shops at alfaelektronik.com.mk (prices in MKD). An XL4015 5 A module was also considered.
- Set the output with a multimeter and nothing connected. Target about **6.5 V**. The repo README says 6.5 V is the minimum for DS3218 servos. Do not exceed about 6.8 V (vendor limit, unverified).
- The repo's BOM pairs the 20 A SZBK07 converter with DS3218 servos and lists the XL4016 9 A for weaker MG996R servos, so an 8 A XL4016 is borderline for 12 DS3218. Watch for rail dips while moving; upgrade if the output sags or the module overheats.
- Wiring advice: 16 to 18 AWG for the power bus, 2x 1000 uF caps across the rail near the PCA9685, shared ground with the Pi, and distribute heavy current with a terminal block instead of the PCA9685 PCB traces (hobby boards are often only rated for 3 to 5 A).
- The repo also lists a relay to cut servo power and an ACS712 current sensor. Both are worth adding.
- USB PD chargers are NOT suitable for the servo rail (3 A per port, motor loads upset PD).

---

## 9. Plan (do not start a phase before the previous one passes its test)

**Phase 0, hardware.** Set the buck output (~6.5 V). Measure the rail at the PCA9685 terminals while standing and sitting. Find the center of mass by balancing the robot on a rod and move heavy items low and central. Put TPU or rubber on the feet. *Done when:* it holds the stand pose for 5 minutes, the rail stays above ~6 V, nothing overheats.

**Phase 1, calibrated kinematics.** Use `kinematics_core.py`. Build a calibration tool and a calibration file mapping geometric joint angle to servo angle for all 12 joints: `servo = offset + direction * geometric_deg` (scale assumed 1.0, verify), plus safe min/max limits. For each joint record the servo angle at two known geometric poses (for example thigh line vertical and horizontal, knee straight and 90 degrees, shoulder vertical and 20 degrees). Also record the collapsed (resting) knee pose as a third reading for `reduced_mode`. Reference the thigh's joint-axis line, not its edge. Use a phone inclinometer, carpenter's square or printed 90 degree jig with the robot propped up so the legs hang free. One calibration file is the single source of truth for the robot and the simulator. Cross-check against `mike4192/spot_micro_kinematics_python` (Python, based on Sen et al. 2017). *Done when:* (a) the new IK stand reproduces the hand-tuned standing set 2 within about 3 degrees, (b) a foot lands within about 5 mm of the commanded point across +/-4 cm (check with a ruler), (c) commanding +/-10 degrees body roll or pitch tilts the body while the feet stay planted.

**Phase 2, control loop.** One 50 Hz loop owns all servo writes; gait, balance and keyboard only set targets. No prints in the loop, float angles, per-servo slew limit, measured frame time (if frames exceed 20 ms write the PCA9685 channels in one burst). Replace accelerometer-only tilt with a complementary filter (gyro rates plus accelerometer) with gyro bias calibrated at startup. Balance becomes a small P/PD controller that adjusts commanded body roll and pitch (about +/-8 degrees authority) through the body-pose IK, never the shoulders directly. Output goes either to the servos or to a WebSocket stream (dry run) so the simulator and Flutter widget show the real controller's angles with no second copy of the kinematics. *Done when:* tilting a board +/-8 degrees under the standing robot keeps the body within about 2 degrees of level without oscillation.

**Phase 3, crawl gait.** Three feet always down. Each step: shift the body so the center of mass sits inside the triangle of the three remaining feet (at least 1.5 cm margin, computed from the measured center of mass, not guessed), lift the foot 3 to 4 cm on a smooth arc, plant it, next leg in the order FL, BR, FR, BL, with the body advancing while all four feet are down. Bring up in stages: simulator first, then robot lifted off the ground to check arcs, then march in place, 1 cm steps, longer steps, turning, backward, WASD mapping. *Done when:* about 10 steps in a straight line on the desk without a fall.

**Phase 4.** Trot with Bezier swing paths; Pi serves velocity commands over WebSocket for the Flutter app; optional Needle 2 as a high-level "walk/sit" layer, never in the control loop.

---

## 10. `reduced_mode` specification

User requirement (user): when `reduced_mode` is `True` the robot walks only using the middle-leg motors (3, 6, 9, 12). The lower-leg motors (4, 7, 10, 13) stay in their collapsed, resting state (4:180, 7:0, 10:180, 13:0) and are never extended.

Reading of the requirement (the user has not explicitly confirmed this): the shoulders (2, 5, 8, 11) also hold a fixed angle, so each leg has exactly one moving joint.

Consequences (derived):
- Body-pose IK cannot be used (a leg with a frozen knee cannot reach arbitrary foot positions). Balance, if any, must be per-joint trims: front versus back thigh angles for pitch.
- With the knee collapsed the "foot" is not the foot-tip point of `kinematics_core.py`. The ground contact is some part of the folded leg (likely the knee end or the flat underside, photo). The effective contact distance from the thigh axis must be measured, then each leg is a rigid piece moving on a fixed arc around the thigh axis.
- Gait design: four swinging legs with phase offsets (crawl order), no foot-trajectory IK. Needs its own simple generator.
- Lower-leg servos must be written once to their collapsed angles and stay powered and locked. Do not release them at start; keep the old shutdown behaviour at exit.
- Simulator and Flutter widget need a joint mask that renders frozen joints.
- Earlier calculations of foot lift for a frozen knee at standing bend (about 10 mm at +/-20 degrees thigh swing) do NOT apply to the collapsed knee.

Open details to confirm with the user: do shoulders stay fixed; what part touches the ground in the collapsed pose; desired thigh swing range.

---

## 11. Open questions and unverified assumptions

1. Knee direction (`KNEE_DIR = -1`, backward) from a photo only.
2. Direction of the 10 mm in-plane offset (`HIP_DROP`, assumed below the shoulder axis) and the L1/L2 assignment.
3. Servo scale (degrees of servo per degree of joint) assumed 1.0.
4. Gyro axis and sign mapping to the robot body.
5. Whether `core_freq=500` and `core_freq_min=500` were set for SPI LEDs.
6. Whether the shoulders are fixed in `reduced_mode`, and the collapsed-pose contact geometry.
7. DS3218MG maximum voltage (about 6.8 V) is from vendor specs, not tested.
8. Needle 2 API: nothing verified.
9. Flutter app structure is unknown beyond "the user has one and wants the simulator in it".

---

## 12. Working agreements with the user

- **Code style (match the user's own scripts):** variables defined at the top; comments written as lowercase `# fn to ...` above functions; variable comments go on the SAME line next to the variable, for example `level_gain = 1.2 # how much correction per degree of tilt`; descriptive `print` statements for steps in setup and test code (but NOT in fast control loops); a `try / except KeyboardInterrupt / except Exception / finally` main block that stops threads, folds the legs and calls `cleanup_servos()` and `turn_off_strip()`. Deliver complete runnable files.

  ```python
  # fn to set the sholders
  def set_sholders():
      print("Running set sholders fn...")
      servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1, STAND_ANGLES[FL_MOTOR_1])
      time.sleep(0.5)
      print("Finished setting sholders.")
      rgb_led_imp.blink_oke()
  ```

- **Safety:** the user once reported a change "almost broke the robot". Test new motion with the robot propped so the legs hang free, move slowly, clamp every joint to calibrated limits, keep the staggered delays when moving many servos at once, check the servo rail voltage, and never command a joint beyond its collapsed or extended limit.
- **Never silently change the working stand behaviour.** Add new behaviour as separate modes or files.
- Be honest about unverified assumptions. Several earlier answers had to be corrected, so treat any claim in this document tagged (unverified) as a hypothesis.
- The user tests quickly on the real robot and reports back with logs and photos. Ask them for concrete measurements or photos when you need them.

---

## 13. References

- `https://github.com/michaelkubina/SpotMicroESP32` : mechanical and electronics base. The README ships no walking software and points to forks that have IK and gaits (an Arduino fork by Blacksheep and SpotMicro-Leika); its BOM recommends DS3218MG servos, TPU foot tips, a relay to cut servo power and a current sensor. Its kinematics folder was not machine-readable (GitHub blocked automated fetches), the user pasted the IK dimensions.
- `https://github.com/mike4192/spot_micro_kinematics_python` : Python quadruped kinematics library based on Sen et al. (2017), "Inverse Kinematic Analysis Of A Quadruped Robot". Configurable hip, upper leg, lower leg, body width and body length.
- `https://github.com/cactus-compute/needle` : optional Needle 2 model.
- `rpi_ws281x` : LED library (SPI mode used).

---

## Appendix A: `kinematics_core.py` (full source)

Pure math, no hardware imports. Run `python3 kinematics_core.py` to execute the self-test. Units: mm and radians. Sign conventions: shoulder > 0 swings the foot outward, thigh > 0 swings the thigh forward, knee > 0 bends the knee, body roll > 0 = left side up, pitch > 0 = nose down, yaw > 0 = nose turns left. These body-sign conventions have not been tied to the gyro's signs yet.

```python
# kinematics core for the robot dog
# pure math - no servo, gyro or gpio imports so it runs on the pi or on a laptop and can feed the simulator
# geometry comes from the SpotMicroESP32 inverse kinematics doc, all lengths in mm, all angles in radians
import math

# leg geometry, measured between the centers of the rotation axes
HIP_OFFSET = 60.5 # sideways distance from the shoulder axis to the leg plane (doc L2)
HIP_DROP = 10.0 # distance inside the leg plane from the shoulder axis down to the thigh axis (doc L1), direction not verified yet
UPPER_LEG = 111.1 # thigh axis to knee axis, straight line (doc L3, the bent part is 107 mm + 30 mm)
LOWER_LEG = 118.5 # knee axis to the center of the foot tip (doc L4)

# body geometry, measured between the shoulder axes
BODY_LENGTH = 207.5 # front shoulder axis to back shoulder axis
BODY_WIDTH = 78.0 # left shoulder axis to right shoulder axis

# which way the knee points, 1 = knee forward, -1 = knee backward, set to backward because the standing photo looks that way, verify on the real robot
KNEE_DIR = -1

# which side of the body every leg is on, 1 = left, -1 = right
LEG_SIDE = {"fl": 1, "fr": -1, "bl": 1, "br": -1}

# shoulder axis position of every leg in the body frame (x forward, y left, z up)
SHOULDER_POS = {
    "fl": ( BODY_LENGTH / 2,  BODY_WIDTH / 2, 0.0),
    "fr": ( BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0),
    "bl": (-BODY_LENGTH / 2,  BODY_WIDTH / 2, 0.0),
    "br": (-BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0),
}

# helper fn to clamp a value between a min and max
def clamp_value(value, min_val, max_val):
    return max(min_val, min(max_val, value))

# fn to solve one leg
# x = forward, y = outward from the shoulder axis (same sign for left and right legs), z = up (negative = below the shoulder axis)
# returns (shoulder, thigh, knee) in radians or None if the foot cannot be reached
# shoulder > 0 swings the foot outward, thigh > 0 swings the thigh forward, knee > 0 bends the knee
def leg_ik(x, y, z):

    # step 1 - the leg plane sits HIP_OFFSET away from the shoulder axis, so only part of the distance is left inside the plane
    r_sq = y * y + z * z
    if r_sq < HIP_OFFSET * HIP_OFFSET:
        return None
    drop = -math.sqrt(r_sq - HIP_OFFSET * HIP_OFFSET) # foot height inside the leg plane, negative = below the shoulder axis
    shoulder = math.atan2(z, y) - math.atan2(drop, HIP_OFFSET)

    # step 2 - two link problem inside the leg plane, measured from the thigh axis
    dx = x # foot forward of the thigh axis
    dz = drop + HIP_DROP # foot height relative to the thigh axis
    reach = math.hypot(dx, dz)
    if reach > UPPER_LEG + LOWER_LEG or reach < abs(UPPER_LEG - LOWER_LEG):
        return None

    # step 3 - law of cosines for the knee and for the thigh
    cos_knee = clamp_value((UPPER_LEG ** 2 + LOWER_LEG ** 2 - reach ** 2) / (2 * UPPER_LEG * LOWER_LEG), -1.0, 1.0)
    knee = math.pi - math.acos(cos_knee) # 0 = straight leg
    cos_beta = clamp_value((UPPER_LEG ** 2 + reach ** 2 - LOWER_LEG ** 2) / (2 * UPPER_LEG * reach), -1.0, 1.0)
    beta = math.acos(cos_beta)
    toward_foot = math.atan2(dx, -dz) # angle of the line to the foot, measured from straight down toward forward
    thigh = toward_foot + KNEE_DIR * beta

    return (shoulder, thigh, knee)

# fn to calculate where the foot is for given joint angles, the opposite of leg_ik
# used to check the ik and to draw the robot in the simulator
def leg_fk(shoulder, thigh, knee):
    shank = thigh - KNEE_DIR * knee # absolute angle of the lower leg
    plane_x = UPPER_LEG * math.sin(thigh) + LOWER_LEG * math.sin(shank)
    plane_z = -HIP_DROP - UPPER_LEG * math.cos(thigh) - LOWER_LEG * math.cos(shank)

    # rotate the leg plane around the shoulder axis
    y = HIP_OFFSET * math.cos(shoulder) - plane_z * math.sin(shoulder)
    z = HIP_OFFSET * math.sin(shoulder) + plane_z * math.cos(shoulder)
    return (plane_x, y, z)

# fn to build a rotation matrix from body roll, pitch and yaw
# roll > 0 = left side up, pitch > 0 = nose down, yaw > 0 = nose turns left
def rotation_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]

# helper fn for matrix times vector
def mat_vec(m, v):
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )

# helper fn for transposed matrix times vector (the inverse of a rotation)
def mat_t_vec(m, v):
    return (
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    )

# fn to get the foot positions for the neutral stance, in the neutral body frame
# height = how far the feet are below the shoulder axes, foot_shift_x moves all feet forward (+) or back (-) relative to the body
# extra_spread moves the feet further out sideways than the leg plane
def neutral_feet(height=180.0, foot_shift_x=0.0, extra_spread=0.0):
    feet = {}
    for leg, (sx, sy, sz) in SHOULDER_POS.items():
        side = LEG_SIDE[leg]
        feet[leg] = (sx + foot_shift_x, sy + side * (HIP_OFFSET + extra_spread), sz - height)
    return feet

# fn to solve all four legs for a body pose while the feet stay planted
# feet_world = planted foot positions {leg: (x, y, z)}, the body is then moved by the translation (tx, ty, tz) and the rotation
# returns {leg: (shoulder, thigh, knee)} or None if any leg cannot reach (the whole pose is rejected so nothing half moves)
def body_pose_ik(feet_world, roll=0.0, pitch=0.0, yaw=0.0, tx=0.0, ty=0.0, tz=0.0):
    r = rotation_matrix(roll, pitch, yaw)
    result = {}
    for leg, foot in feet_world.items():
        # the foot seen from the moved body
        fb = mat_t_vec(r, (foot[0] - tx, foot[1] - ty, foot[2] - tz))
        sx, sy, sz = SHOULDER_POS[leg]
        side = LEG_SIDE[leg]
        angles = leg_ik(fb[0] - sx, side * (fb[1] - sy), fb[2] - sz) # mirror y so outward is positive on both sides
        if angles is None:
            return None
        result[leg] = angles
    return result

# fn to test the math, run this file on its own to call it
def run_self_test():
    print("Test 1 - leg ik then fk must land on the same point...")
    worst = 0.0
    solved = 0
    unreachable = 0
    for x in range(-60, 61, 20):
        for y in range(30, 111, 10):
            for z in range(-215, -139, 5):
                angles = leg_ik(x, y, z)
                if angles is None:
                    unreachable += 1
                    continue
                fx, fy, fz = leg_fk(*angles)
                worst = max(worst, abs(fx - x), abs(fy - y), abs(fz - z))
                solved += 1
    print("  solved : " + str(solved) + ", unreachable : " + str(unreachable) + ", worst error : " + str(worst) + " mm")

    print("Test 2 - feet must stay planted while the body rolls, pitches, yaws and moves...")
    feet = neutral_feet(180.0)
    poses = [
        ("roll +10",  dict(roll=math.radians(10))),
        ("roll -10",  dict(roll=math.radians(-10))),
        ("pitch +10", dict(pitch=math.radians(10))),
        ("pitch -10", dict(pitch=math.radians(-10))),
        ("yaw +12",   dict(yaw=math.radians(12))),
        ("shift x +25, y -15", dict(tx=25.0, ty=-15.0)),
        ("lower 30",  dict(tz=-30.0)),
        ("mixed",     dict(roll=math.radians(6), pitch=math.radians(-5), yaw=math.radians(4), tx=10.0, tz=-10.0)),
    ]
    for name, pose in poses:
        angles = body_pose_ik(feet, **pose)
        if angles is None:
            print("  " + name + " > rejected (a leg cannot reach)")
            continue
        r = rotation_matrix(pose.get("roll", 0.0), pose.get("pitch", 0.0), pose.get("yaw", 0.0))
        t = (pose.get("tx", 0.0), pose.get("ty", 0.0), pose.get("tz", 0.0))
        worst = 0.0
        for leg, a in angles.items():
            fx, fy, fz = leg_fk(*a)
            sx, sy, sz = SHOULDER_POS[leg]
            fb = (sx + fx, sy + LEG_SIDE[leg] * fy, sz + fz) # foot in the moved body frame
            w = mat_vec(r, fb)
            world = (w[0] + t[0], w[1] + t[1], w[2] + t[2])
            worst = max(worst, abs(world[0] - feet[leg][0]), abs(world[1] - feet[leg][1]), abs(world[2] - feet[leg][2]))
        print("  " + name + " > worst foot drift : " + str(worst) + " mm")

    print("Neutral stance at 180 mm height (degrees, shoulder / thigh / knee)...")
    for leg, a in body_pose_ik(feet).items():
        print("  " + leg + " > " + str(round(math.degrees(a[0]), 1)) + " / " + str(round(math.degrees(a[1]), 1)) + " / " + str(round(math.degrees(a[2]), 1)))

if __name__ == "__main__":
    run_self_test()
```

---

## Appendix B: where the numbers in section 7 came from

- Gyro readings were produced by the user running a small script that printed `gyro_imp.get_pitch_roll()` five times, 0.5 s apart, once with the robot flat on the table and once standing.
- Standing sets 1 and 2 are the final values from two manual calibration sessions where the user typed servo indexes, angles and range 180 into a jog script and watched the robot.
- The IK offsets in the OLD `kinematics_imp.py` were back-solved from standing set 2 and should be considered invalid.
