# Constants
RATE = 30

# Robot Command Dictionary
commands = {'forward' : 0b100000000000, 'reverse' : 0b010000000000, 'left' : 0b001000000000, 'right' : 0b000100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b000000000010, 'back_dump' : 0b000000000001,
                'arms_up' : 0b000010100000, 'arms_down' : 0b000001010000, 'null': 0b000000000000}