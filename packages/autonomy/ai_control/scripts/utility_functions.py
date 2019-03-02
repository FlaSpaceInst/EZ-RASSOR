#!/usr/bin/env python
import rospy

def set_front_arm_angle(world_state, target_angle):
    """ Set front arm to absolute angle target_angle in radians. """

    if target_angle > world_state.state_flags['front_arm_angle']:
        while(target_angle > world_state.state_flags['front_arm_angle']):
            command_pub.publish(commands['front_arm_up'])
            rate.sleep()
    else:
        while(target_angle < world_state.state_flags['front_arm_angle']):
            command_pub.publish(commands['front_arm_down'])
            rate.sleep()

    
    command_pub.publish(commands['null'])


def set_back_arm_angle(world_state, target_angle):
    """ Set back arm to absolute angle target_angle in radians. """

    if target_angle > world_state.state_flags['back_arm_angle']:
        while(target_angle > world_state.state_flags['back_arm_angle']):
            command_pub.publish(commands['back_arm_up'])
            rate.sleep()
    else:
        while(target_angle < world_state.state_flags['back_arm_angle']):
            command_pub.publish(commands['back_arm_down'])
            rate.sleep()

    command_pub.publish(commands['null'])