# robot_pi/cleanup.py
"""Centralized hardware cleanup and release module for the Quadruped Robot.

Releases PCA9685 servos, turns off WS281x RGB LED strip, silences tonal buzzer,
stops audio playback, and safely resets GPIO lines.
"""

import logging

logger = logging.getLogger("robot.cleanup")


def cleanup_all_hardware():
    """Safely cleans up and releases all physical hardware devices."""
    print("==========================================")
    print(">>> Executing Robot Hardware Cleanup <<<")
    print("==========================================")

    # 1. Release PCA9685 servo motors across all 16 channels
    try:
        from drivers import servo_driver_imp
        servo_driver_imp.cleanup_servos()
    except Exception as e:
        logger.debug("PCA9685 servo cleanup skipped or failed: %s", e)

    # 2. Turn off WS281x RGB LED strip / plate
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.turn_off_strip()
    except Exception as e:
        logger.debug("RGB LED strip cleanup skipped or failed: %s", e)

    # 3. Stop GPIO 23 tonal buzzer if buzzing
    try:
        from drivers import mini_speaker_imp
        if hasattr(mini_speaker_imp, "tb") and mini_speaker_imp.tb is not None:
            mini_speaker_imp.tb.stop()
    except Exception as e:
        logger.debug("Buzzer cleanup skipped or failed: %s", e)

    # 4. Stop speech synthesis & audio playback
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.stop_audio()
    except Exception as e:
        logger.debug("Audio playback cleanup skipped or failed: %s", e)

    # 5. Standalone single-servo GPIO 24 cleanup (if initialized by servo_imp)
    try:
        import sys
        if "drivers.servo_imp" in sys.modules:
            from drivers import servo_imp
            servo_imp.servo_cleanup()
    except Exception as e:
        logger.debug("Single servo cleanup skipped: %s", e)

    print("Hardware cleanup completed successfully.")
