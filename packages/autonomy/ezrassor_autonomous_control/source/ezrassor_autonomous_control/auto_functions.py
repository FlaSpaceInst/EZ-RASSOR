import rospy
import math
from ai_objects import WorldState, ROSUtility
from utility_functions import self_check, reverse_turn 
from nav_functions import calculate_heading, adjust_angle

def auto_drive(world_state, ros_util):
    """ Travel forward in a straight line. Avoid obstacles while maintaining heading. """
    # Main loop until command is canceled
    while ros_util.auto_function_command != 0:

        # Avoid obstacles by turning left or right if warning flag is raised
        while world_state.state_flags['warning_flag'] == 1:
            ros_util.command_pub.publish(ros_util.commands['right'])
            ros_util.rate.sleep()
        while world_state.state_flags['warning_flag'] == 2:
            ros_util.command_pub.publish(ros_util.commands['left'])
            ros_util.rate.sleep()
        
        if world_state.state_flags['warning_flag'] == 3:
            reverse_turn(world_state, ros_util)

        ros_util.command_pub.publish(ros_util.commands['forward'])
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])
    ros_util.command_pub.publish(ros_util.kill_bit)

def auto_drive_location(world_state, ros_util):
    """ Navigate to location. Avoid obstacles while moving toward location. """
    print("Auto Driving to {}".format(world_state.state_flags['target_location']))
    # Main loop until location is reached
    while world_state.state_flags['positionX'] != world_state.state_flags['target_location'][0] and world_state.state_flags['positionY'] != world_state.state_flags['target_location'][1]:
        # Get new heading angle relative to current heading as (0,0)
        new_heading = calculate_heading(world_state, ros_util)

        # Calculate angle difference needed to adjust heading
        angle_difference = adjust_angle(world_state.state_flags['heading'], new_heading)

        if angle_difference > 0:
            direction = 'left'
        else:
            direction = 'right'

        # Adjust heading until it matches new heading
        while not ((new_heading - 1) < world_state.state_flags['heading'] < (new_heading + 1)):
            print(world_state.state_flags['heading'], new_heading)
            ros_util.command_pub.publish(ros_util.commands[direction])
            ros_util.rate.sleep()

        # Avoid obstacles by turning left or right if warning flag is raised
        while(world_state.state_flags['warning_flag'] == 1):
            ros_util.command_pub.publish(ros_util.commands['right'])
            ros_util.rate.sleep()
        while(world_state.state_flags['warning_flag'] == 2):
            ros_util.command_pub.publish(ros_util.commands['left'])
            ros_util.rate.sleep()

        if world_state.state_flags['warning_flag'] == 3:
            reverse_turn(world_state, ros_util)

        # Otherwise go forward
        ros_util.command_pub.publish(ros_util.commands['forward'])
            
        ros_util.rate.sleep()
    
    ros_util.command_pub.publish(ros_util.commands['null'])
    ros_util.command_pub.publish(ros_util.kill_bit)
        
def auto_dig(world_state, ros_util, duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """
    print("Auto Digging for {} Seconds".format(duration))
    t = 0
    while t < duration*30:        
        ros_util.command_pub.publish(ros_util.commands['forward'] | ros_util.commands['front_dig'] | ros_util.commands['back_dig'])
        t+=1
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])
    ros_util.command_pub.publish(ros_util.kill_bit)


def auto_dock(world_state, ros_util):
    """ Dock with the hopper. """
    print("Auto Returning to {}".format([0,0]))
    world_state.state_flags['target_location'] = [0,0]
    auto_drive_location(world_state, ros_util)

def auto_dump(world_state, ros_util, duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """
    print("Auto Dumping")
    t = 0
    while t < duration*30:        
        ros_util.command_pub.publish(ros_util.commands['front_dump'])
        t+=1
        ros_util.rate.sleep()
    t = 0

    new_heading = world_state.state_flags['heading'] = (world_state.state_flags['heading'] - 180) % 360

    while not ((new_heading - 1) < world_state.state_flags['heading'] < (new_heading + 1)):
            ros_util.command_pub.publish(ros_util.commands['left'])
            ros_util.rate.sleep()

    while t < duration*30:        
        ros_util.command_pub.publish(ros_util.commands['front_dump'])
        t+=1
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])
    ros_util.command_pub.publish(ros_util.kill_bit)