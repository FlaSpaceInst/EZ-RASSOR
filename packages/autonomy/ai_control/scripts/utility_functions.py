#!/usr/bin/env python
import rospy

def set_front_arm_angle(world_state, ros_util, target_angle):
    """ Set front arm to absolute angle target_angle in radians. """

    ros_util.status_pub.publish("Setting Front Arm Angle to {} Radians".format(target_angle))
    if target_angle > world_state.state_flags['front_arm_angle']:
        while(target_angle > world_state.state_flags['front_arm_angle']):
            ros_util.command_pub.publish(ros_util.commands['front_arm_up'])
            ros_util.rate.sleep()
    else:
        while(target_angle < world_state.state_flags['front_arm_angle']):
            ros_util.command_pub.publish(ros_util.commands['front_arm_down'])
            ros_util.rate.sleep()

    
    ros_util.command_pub.publish(ros_util.commands['null'])


def set_back_arm_angle(world_state, ros_util, target_angle):
    """ Set back arm to absolute angle target_angle in radians. """

    ros_util.status_pub.publish("Setting Back Arm Angle to {} Radians".format(target_angle))
    if target_angle > world_state.state_flags['back_arm_angle']:
        while(target_angle > world_state.state_flags['back_arm_angle']):
            ros_util.command_pub.publish(ros_util.commands['back_arm_up'])
            ros_util.rate.sleep()
    else:
        while(target_angle < world_state.state_flags['back_arm_angle']):
            ros_util.command_pub.publish(ros_util.commands['back_arm_down'])
            ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])

def initial_check(world_state, ros_util):
    """  """
    pass

def battery_check():
    """  """
    pass