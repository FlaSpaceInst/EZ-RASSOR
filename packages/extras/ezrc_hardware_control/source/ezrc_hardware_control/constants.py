"""Some package-level constants for the ezrc_hardware_control ROS package.

Written by Tiger Sachse.
"""
import RPi.GPIO


GPIO_MODE = GPIO.BCM
DRIVER_FREQUENCY = 60
ENABLE_GPIO_WARNINGS = False
MESSAGE_FORMAT = "EZRC_HARDWARE_CONTROL ({0}): {1}."
MOVEMENT_TOGGLES_TOPIC = "/ezrassor/movement_toggles"
