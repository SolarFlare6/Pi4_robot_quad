# robot_pi/navigation/locomotion_placeholders.py
"""Locomotion, walking, and posing placeholder functions for the Quadruped Robot.

These functions provide clean entry points for the robot's physical locomotion.
Implement your custom inverse kinematics, gait timing, and leg trajectories here.
"""

import logging

logger = logging.getLogger("robot.locomotion")


def walk_forward(distance: float = 1.0, speed: float = 1.0) -> None:
    """Moves the robot forward by the specified distance."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] walk_forward called: distance={distance}, speed={speed}")


def walk_backward(distance: float = 1.0, speed: float = 1.0) -> None:
    """Moves the robot backward by the specified distance."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] walk_backward called: distance={distance}, speed={speed}")


def walk_left(distance: float = 1.0, speed: float = 1.0) -> None:
    """Steps the robot laterally to the left."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] walk_left called: distance={distance}, speed={speed}")


def walk_right(distance: float = 1.0, speed: float = 1.0) -> None:
    """Steps the robot laterally to the right."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] walk_right called: distance={distance}, speed={speed}")


def turn_left(angle: float = 45.0) -> None:
    """Rotates the robot counter-clockwise (left) by the specified angle in degrees."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] turn_left called: angle={angle} deg")


def turn_right(angle: float = 45.0) -> None:
    """Rotates the robot clockwise (right) by the specified angle in degrees."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] turn_right called: angle={angle} deg")


def walk(direction: str = "forward", distance: float = 1.0) -> None:
    """General walking dispatcher."""
    d = str(direction).lower().strip()
    if d in ("forward", "fwd"):
        walk_forward(distance)
    elif d in ("backward", "back", "bwd"):
        walk_backward(distance)
    elif d == "left":
        walk_left(distance)
    elif d == "right":
        walk_right(distance)
    else:
        print(f"[LOCOMOTION] Unknown walking direction: {direction}")


def turn(direction: str = "left", angle: float = 45.0) -> None:
    """General turning dispatcher."""
    d = str(direction).lower().strip()
    if d == "left":
        turn_left(angle)
    elif d == "right":
        turn_right(angle)
    else:
        print(f"[LOCOMOTION] Unknown turning direction: {direction}")


def stand() -> None:
    """Moves the robot body into standing posture using calibrated stand angles."""
    # [USER IMPLEMENTATION BLOCK]
    print("[LOCOMOTION] stand called (transitioning legs to standing angles)")


def sit() -> None:
    """Moves the robot body into resting / sitting posture."""
    # [USER IMPLEMENTATION BLOCK]
    print("[LOCOMOTION] sit called (transitioning legs to resting angles)")


def stop() -> None:
    """Halts current gait cycle and holds current posture."""
    # [USER IMPLEMENTATION BLOCK]
    print("[LOCOMOTION] stop called")


def emergency_stop() -> None:
    """Immediately halts all locomotion, releases servo coils, and cleans up hardware."""
    # [USER IMPLEMENTATION BLOCK]
    print("[LOCOMOTION] emergency_stop called (halting motors and executing hardware cleanup)")
    try:
        from cleanup import cleanup_all_hardware
        cleanup_all_hardware()
    except Exception as exc:
        logger.debug("Hardware cleanup in emergency_stop failed: %s", exc)


def set_pose(pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0, height: float = 0.0) -> None:
    """Adjusts body height and orientation angles via kinematic posing."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] set_pose called: pitch={pitch}, roll={roll}, yaw={yaw}, height={height}")


def set_gait(mode: str = "walk") -> None:
    """Configures gait timing preset (e.g. walk, trot, crawl)."""
    # [USER IMPLEMENTATION BLOCK]
    print(f"[LOCOMOTION] set_gait called: mode={mode}")
