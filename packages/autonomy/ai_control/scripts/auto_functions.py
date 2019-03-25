#!/usr/bin/env python
import rospy
import math

def auto_drive(world_state, ros_util):
    """ Travel forward in a straight line. Avoid obstacles while maintaining heading. """
    
    while ros_util.auto_function_command != 0:
        while(world_state.state_flags['warning_flag'] == 1):
            ros_util.command_pub.publish(ros_util.commands['right'])
            ros_util.rate.sleep()
        while(world_state.state_flags['warning_flag'] == 2):
            ros_util.command_pub.publish(ros_util.commands['left'])
            ros_util.rate.sleep()
        
        ros_util.command_pub.publish(ros_util.commands['forward'])
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])

def auto_drive_location(world_state, ros_util):
    """ Navigate to location. Avoid obstacles while moving toward location. """

    while world_state.state_flags['positionX'] != world_state.state_flags['target_location'][0] and world_state.state_flags['positionY'] != world_state.state_flags['target_location'][1]:
        print(world_state.state_flags['positionX'], world_state.state_flags['positionY'])
        new_heading = math.atan2( (world_state.state_flags['target_location'][0] - world_state.state_flags['positionY']), (world_state.state_flags['target_location'][1] - world_state.state_flags['positionX']) )
        new_heading = int(360*new_heading/math.pi)
        ros_util.status_pub.publish("New Heading: {}".format(new_heading))

        angle_difference = new_heading - world_state.state_flags['heading']
        angle_difference = (angle_difference + 180) % 360 - 180

        if angle_difference > 0:
            direction = 'left'
            delta = 1
        else:
            direction = 'right'
            delta = -1

        while not ((new_heading - 1) < world_state.state_flags['heading'] < (new_heading + 1)):
            ros_util.command_pub.publish(ros_util.commands[direction])
            world_state.state_flags['heading'] += delta
            print(world_state.state_flags['heading'], new_heading)
            ros_util.rate.sleep()

        ros_util.command_pub.publish(ros_util.commands['forward'])
            
        ros_util.rate.sleep()
    
    ros_util.command_pub.publish(ros_util.commands['null'])
        
def auto_dig(world_state, ros_util, duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """

    while ros_util.auto_function_command != 0:
        while(world_state.state_flags['warning_flag_front'] == 1):
            ros_util.command_pub.publish(ros_util.commands['right'])
            ros_util.rate.sleep()
        while(world_state.state_flags['warning_flag_front'] == 2):
            ros_util.command_pub.publish(ros_util.commands['left'])
            ros_util.rate.sleep()
        
        ros_util.command_pub.publish(ros_util.commands['forward'] | ros_util.commands['front_dig'] | ros_util.commands['back_dig'])
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])

def auto_dock(world_state, ros_util):
    """ Dock with the hopper. """
    pass

def auto_self_right(world_state, ros_util):
    """  """
    pass