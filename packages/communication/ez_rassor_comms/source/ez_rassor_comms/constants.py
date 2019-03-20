"""Some package-level constants for the ez_rassor_comms ROS package.

Written by Tiger Sachse.
"""
QUEUE_SIZE = 10
AI_KILL_MASK = 0b1000000000000
AI_TOGGLES_MASK = 0b1111000000000000
MOVEMENT_TOGGLES_MASK = 0b0000111111111111
REQUESTS_TOPIC = "/ezrassor/requests"
ROUTINE_TOGGLES_TOPIC = "/ezrassor/routine_toggles"
MOVEMENT_TOGGLES_TOPIC = "/ezrassor/movement_toggles"
ROUTINE_RESPONSES_TOPIC = "/ezrassor/routine_responses"
