#!/usr/bin/env python
import rospy
import math

def auto_drive(world_state, ros_util):
    """ Travel forward in a straight line. Avoid obstacles while maintaining heading. """
    # Main loop until command is canceled
    while ros_util.auto_function_command != 0:

        # Avoid obstacles by turning left or right if warning flag is raised
        while(world_state.state_flags['warning_flag'] == 1):
            ros_util.command_pub.publish(ros_util.commands['right'])
            ros_util.rate.sleep()
        while(world_state.state_flags['warning_flag'] == 2):
            ros_util.command_pub.publish(ros_util.commands['left'])
            ros_util.rate.sleep()
        
        ros_util.command_pub.publish(ros_util.commands['forward'])
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])
    ros_util.command_pub.publish(ros_util.commands['kill_bit'])

def auto_drive_location(world_state, ros_util):
    """ Navigate to location. Avoid obstacles while moving toward location. """

    # Main loop until location is reached
    while world_state.state_flags['positionX'] != world_state.state_flags['target_location'][0] and world_state.state_flags['positionY'] != world_state.state_flags['target_location'][1]:
        print(world_state.state_flags['positionX'], world_state.state_flags['positionY'])
        # Get new heading angle relative to current heading as (0,0)
        new_heading = math.atan2( (world_state.state_flags['target_location'][0] - world_state.state_flags['positionY']), (world_state.state_flags['target_location'][1] - world_state.state_flags['positionX']) )
        new_heading = int(360*new_heading/math.pi)
        ros_util.status_pub.publish("New Heading: {}".format(new_heading))

        # Calculate angle difference needed to adjust heading
        angle_difference = new_heading - world_state.state_flags['heading']
        angle_difference = (angle_difference + 180) % 360 - 180

        if angle_difference > 0:
            direction = 'left'
            delta = .5
        else:
            direction = 'right'
            delta = -.5

        # Adjust heading until it matches new heading
        while not ((new_heading - 1) < world_state.state_flags['heading'] < (new_heading + 1)):
            print(world_state.state_flags['heading'], new_heading)
            ros_util.command_pub.publish(ros_util.commands[direction])
            world_state.state_flags['heading'] += delta # Temporary until we fix heading calculation using visual odometry
            ros_util.rate.sleep()

        # Avoid obstacles by turning left or right if warning flag is raised
        while(world_state.state_flags['warning_flag'] == 1):
            ros_util.command_pub.publish(ros_util.commands['right'])
            world_state.state_flags['heading'] += delta # Temporary until we fix heading calculation using visual odometry
            ros_util.rate.sleep()
        while(world_state.state_flags['warning_flag'] == 2):
            ros_util.command_pub.publish(ros_util.commands['left'])
            world_state.state_flags['heading'] += delta # Temporary until we fix heading calculation using visual odometry
            ros_util.rate.sleep()
        
        # Otherwise go forward
        ros_util.command_pub.publish(ros_util.commands['forward'])
            
        ros_util.rate.sleep()
    
    ros_util.command_pub.publish(ros_util.commands['null'])
    ros_util.command_pub.publish(ros_util.commands['kill_bit'])
        
def auto_dig(world_state, ros_util, duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """

    while ros_util.auto_function_command != 0:        
        ros_util.command_pub.publish(ros_util.commands['forward'] | ros_util.commands['front_dig'] | ros_util.commands['back_dig'])
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])
    ros_util.command_pub.publish(ros_util.commands['kill_bit'])


def auto_dock(world_state, ros_util):
    """ Dock with the hopper. """
    world_state.state_flags['target_location'] = [0,0]
    auto_drive_location(world_state, ros_util)

def auto_self_right(world_state, ros_util):
    """  """
    pass