# robot_pi/init_hooks.py
"""One-time startup initialization hook for the Quadruped Robot.

This script executes exactly once when the robot application initializes.
You can implement custom hardware setup, calibration, greeting sequences, or self-tests here.
"""

import logging
from typing import Any, Dict

logger = logging.getLogger("robot.init_hooks")


def initialize_robot(context: Dict[str, Any] = None) -> bool:
    """Executes one-time startup sequence for the robot.

    Args:
        context: Optional dictionary containing references to active drivers,
                 pan_tilt controller, servo kit, etc.

    Returns:
        bool: True if initialization completed successfully.
    """
    logger.info("==========================================")
    logger.info(">>> Initializing Quadruped Robot Hardware <<<")
    logger.info("==========================================")

    # -------------------------------------------------------------------------
    # [USER IMPLEMENTATION BLOCK]
    # Place your custom startup logic below (e.g. self-test routine,
    # calibration checks, playing a startup sound, setting initial posture, etc.)
    # -------------------------------------------------------------------------

    try:
        # Example default action: ensure Pan/Tilt servos are centered to 90 degrees
        if context and "pan_tilt" in context:
            pan_tilt = context["pan_tilt"]
            pan_tilt.center()
            logger.info("Camera pan/tilt servos centered to (90, 90).")

        # Visual feedback: flash green on RGB led strip if available
        if context and "drivers" in context and "rgb_led" in context["drivers"]:
            rgb_led = context["drivers"]["rgb_led"]
            try:
                rgb_led.blink_oke()
            except Exception as e:
                logger.debug("RGB LED startup blink skipped: %s", e)

        print("[INIT] Robot initialization hook executed successfully.")
        return True

    except Exception as exc:
        logger.error("Error during robot startup initialization: %s", exc, exc_info=True)
        return False
