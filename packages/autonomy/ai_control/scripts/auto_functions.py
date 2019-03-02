#!/usr/bin/env python
import rospy

def auto_drive(world_state, rospy_config):
    """ Travel forward in a straight line. Avoid obstacles while maintaining heading. """
    
    while world_state.auto_function_command != 0:
        while(world_state.state_flags['warning_flag'] == 1):
            rospy_config.command_pub.publish(rospy_config.commands['right'])
            rospy_config.rate.sleep()
        while(world_state.state_flags['warning_flag'] == 2):
            rospy_config.command_pub.publish(rospy_config.commands['left'])
            rospy_config.rate.sleep()
        
        rospy_config.command_pub.publish(rospy_config.commands['forward'])
        rospy_config.rate.sleep()

    rospy_config.command_pub.publish(rospy_config.commands['null'])

def auto_drive_location(world_state, rospy_config, location):
    """ Navigate to location. Avoid obstacles while moving toward location. """
    pass


def auto_dig(world_state, rospy_config, duration):
    """ Rotate both drums inward and drive forward for duration time in seconds. """

    while world_state.auto_function_command != 0:
        while(world_state.state_flags['warning_flag_front'] == 1):
            rospy_config.command_pub.publish(rospy_config.commands['right'])
            rospy_config.rate.sleep()
        while(world_state.state_flags['warning_flag_front'] == 2):
            rospy_config.command_pub.publish(rospy_config.commands['left'])
            rospy_config.rate.sleep()
        
        rospy_config.command_pub.publish(rospy_config.commands['forward'] | rospy_config.commands['front_dig'] | rospy_config.commands['back_dig'])
        rospy_config.rate.sleep()

    rospy_config.command_pub.publish(rospy_config.commands['null'])

def auto_dock(world_state, rospy_config):
    """ Dock with the hopper. """
    pass

def auto_self_right(world_state, rospy_config):
    """  """
    pass