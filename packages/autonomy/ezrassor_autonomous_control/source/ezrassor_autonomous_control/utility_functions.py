import rospy
import time

def set_front_arm_angle(world_state, ros_util, target_angle):
    """ Set front arm to absolute angle target_angle in radians. """

    ros_util.status_pub.publish("Setting Front Arm Angle to {} Radians".format(target_angle))
    if target_angle > world_state.state_flags['front_arm_angle']:
        while target_angle > world_state.state_flags['front_arm_angle']:
            ros_util.command_pub.publish(ros_util.commands['front_arm_up'])
            ros_util.rate.sleep()
    else:
        while target_angle < world_state.state_flags['front_arm_angle']:
            ros_util.command_pub.publish(ros_util.commands['front_arm_down'])
            ros_util.rate.sleep()

    
    ros_util.command_pub.publish(ros_util.commands['null'])


def set_back_arm_angle(world_state, ros_util, target_angle):
    """ Set back arm to absolute angle target_angle in radians. """

    ros_util.status_pub.publish("Setting Back Arm Angle to {} Radians".format(target_angle))
    if target_angle > world_state.state_flags['back_arm_angle']:
        while target_angle > world_state.state_flags['back_arm_angle']:
            ros_util.command_pub.publish(ros_util.commands['back_arm_up'])
            ros_util.rate.sleep()
    else:
        while target_angle < world_state.state_flags['back_arm_angle']:
            ros_util.command_pub.publish(ros_util.commands['back_arm_down'])
            ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])

def initial_check(world_state, ros_util):
    """  """
    pass

def reverse_turn(world_state, ros_util):
    """ Reverse until object no longer detected and turn left """

    while world_state.state_flags['warning_flag'] == 3:
        ros_util.command_pub.publish(ros_util.commands['reverse'])
        ros_util.rate.sleep()

    new_heading = world_state.state_flags['heading'] + 60

    while (new_heading - 1) < world_state.state_flags['heading'] < (new_heading + 1):
        ros_util.command_pub.publish(ros_util.commands['left'])

def self_right_from_side(world_state, ros_util):
    """ Flip EZ-RASSOR over from its side. """

    self_right_execution_time = 0.5
    start_time = time.time()
    ros_util.status_pub.publish("Initiating Self Right")
    while(time.time() - start_time < self_right_execution_time):
        if world_state.state_flags['on_side'] == False:
            ros_util.command_pub.publish(ros_util.commands['null'])
            return
        ros_util.command_pub.publish(ros_util.commands['back_arm_up'])
        ros_util.command_pub.publish(ros_util.commands['front_arm_up'])
            
    ros_util.command_pub.publish(ros_util.commands['null'])