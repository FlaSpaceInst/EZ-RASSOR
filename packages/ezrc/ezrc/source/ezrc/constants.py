"""All configuration constants for the EZRC ROS package.

Written by Tiger Sachse.
"""
import RPi.GPIO

# General constants.
GPIO_MODE = GPIO.BCM
DRIVER_FREQUENCY = 60
ENABLE_GPIO_WARNINGS = False
MESSAGE_FORMAT = "EZRC ({0}): %s."
MOVEMENT_TOGGLES_TOPIC = "/ezrassor/movement_toggles"

# Arms node constants.
ARM_NODE_NAME = "arms_driver"
ARM_MASK = 0b000011110000
ARM_SHIFT_AMOUNT = 5
REAR_ARM_CHANNEL = 12
FORWARD_ARM_CHANNEL = 15
ARM_SLEEP_DURATION = .012
REAR_ARM_GROUND_WAVELENGTH = 325
REAR_ARM_VERTICAL_WAVELENGTH = 700
FORWARD_ARM_GROUND_WAVELENGTH = 675
FORWARD_ARM_VERTICAL_WAVELENGTH = 325
ARM_HALT_MESSAGE = "Stopping both arms"
ARM_DEBUGGING_MESSAGES = (
    "Moving forward arm up",
    "Moving forward arm down",
    "Moving rear arm up",
    "Moving rear arm down",
)

# Drums node constants.
DRUM_NODE_NAME = "drums_driver"
DRUM_MASK = 0b000000001111
DRUM_SLEEP_DURATION = .2
REAR_DRUM_PINS = (13, 19, 26)
FORWARD_DRUM_PINS = (20, 21, 16)
DRUM_HALT_MESSAGE = "Stopping both drums"
DRUM_DEBUGGING_MESSAGES = (
    "Digging with forward drum",
    "Dumping from forward drum",
    "Digging with rear drum",
    "Dumping from rear drum",
)

# Wheels node constants.
WHEEL_NODE_NAME = "wheels_driver"
WHEEL_MASK = 0b111100000000
WHEEL_SPEED = 2000
LEFT_WHEEL_CHANNEL = 5
RIGHT_WHEEL_CHANNEL = 4
LEFT_WHEEL_PINS = (17, 18)
RIGHT_WHEEL_PINS = (22, 27)
WHEEL_HALT_MESSAGE = "Stopping the wheels"
WHEEL_DEBUGGING_MESSAGES = (
    "Driving left side forward",
    "Driving left side backward",
    "Driving right side forward",
    "Driving right side backward",
)
