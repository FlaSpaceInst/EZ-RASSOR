import rospy
import time
import nav_functions as nf

def set_front_arm_angle(world_state, ros_util, target_angle):
    """ Set front arm to absolute angle target_angle in radians. """
    rospy.loginfo('Setting front arm angle to %s radian%s...',
                  str(target_angle),
                  "" if target_angle == 1 else "s")

    if target_angle > world_state.front_arm_angle:
        while target_angle > world_state.front_arm_angle:
            ros_util.publish_actions('stop', 1, 0, 0, 0)
            ros_util.rate.sleep()
    else:
        while target_angle < world_state.front_arm_angle:
            ros_util.publish_actions('stop', -1, 0, 0, 0)
            ros_util.rate.sleep()

    ros_util.publish_actions('stop', 0, 0, 0, 0)

def set_back_arm_angle(world_state, ros_util, target_angle):
    """ Set back arm to absolute angle target_angle in radians. """
    rospy.loginfo('Setting back arm angle to %s radian%s...',
                  str(target_angle),
                  "" if target_angle == 1 else "s")

    if target_angle > world_state.back_arm_angle:
        while target_angle > world_state.back_arm_angle:
            ros_util.publish_actions('stop', 0, 1, 0, 0)
            ros_util.rate.sleep()
    else:
        while target_angle < world_state.back_arm_angle:
            ros_util.publish_actions('stop', 0, -1, 0, 0)
            ros_util.rate.sleep()

    ros_util.publish_actions('stop', 0, 0, 0, 0)

def self_check(world_state, ros_util):
    """ Check for unfavorable states in the system 
        and handle or quit gracefully. 
    """
    if ros_util.auto_function_command == 32 or ros_util.auto_function_command == 0:
        rospy.loginfo("Cancelling auto-function command...")
        ros_util.publish_actions('stop', 0, 0, 0, 0)
        ros_util.control_pub.publish(False)
        return -1
    # Future status checks for physical hardware
    '''
    if world_state.on_side == True:
        rospy.loginfo("On side! Attempting auto self-right...")
        return 2
    if world_state.battery < 10:
        rospy.loginfo("Low battery! Returning to origin...")
        world_state.target_location = [0,0]
        return 3
    if world_state.hardware_status == False:
        rospy.loginfo("Hardware failure! Shutting down...")
        ros_util.publish_actions('stop', 1, 0, 0, 0)
        ros_util.control_pub.publish(False)
        return -1
    '''
    return 1

def turn(new_heading, direction, world_state, ros_util):

    # Adjust heading until it matches new heading
    while not ((new_heading - 2) < world_state.heading < (new_heading + 2)):
        ros_util.publish_actions(direction, 0, 0, 0, 0)
        ros_util.rate.sleep()

def reverse_turn(world_state, ros_util):
    """ Reverse until object no longer detected and turn left """

    while world_state.warning_flag == 3:
        ros_util.publish_actions('reverse', 0, 0, 0, 0)
        ros_util.rate.sleep()

    new_heading = (world_state.heading + 60) % 360

    while (new_heading - 1) < world_state.heading < (new_heading + 1):
        ros_util.publish_actions('left', 0, 0, 0, 0)


def dodge_left(world_state, ros_util):
    start_x = world_state.positionX
    start_y = world_state.positionY

    threshold = 0

    while world_state.warning_flag != 0 or (threshold < 25):
        if world_state.warning_flag == 0:
            threshold+=1
        ros_util.publish_actions('left', 0, 0, 0, 0)
        ros_util.rate.sleep()

    while nf.euclidean_distance(start_x, world_state.positionX, 
                                start_y, world_state.positionY) < 2:
        ros_util.publish_actions('forward', 0, 0, 0, 0)
        ros_util.rate.sleep()


def dodge_right(world_state, ros_util):
    start_x = world_state.positionX
    start_y = world_state.positionY

    threshold = 0

    while world_state.warning_flag != 0 or (threshold < 25):
        if world_state.warning_flag == 0:
            threshold+=1
        ros_util.publish_actions('right', 0, 0, 0, 0)
        ros_util.rate.sleep()

    while nf.euclidean_distance(start_x, world_state.positionX, 
                                start_y, world_state.positionY) < 2:
        
        ros_util.publish_actions('forward', 0, 0, 0, 0)
        ros_util.rate.sleep()

def self_right_from_side(world_state, ros_util):
    """ Flip EZ-RASSOR over from its side. """

    rospy.loginfo("Starting auto self-right...")
    while(world_state.on_side != False):
        ros_util.publish_actions('stop', 0, 1, 0, 0)
        ros_util.publish_actions('stop', 1, 0, 0, 0)
            
    ros_util.publish_actions('stop', 0, 0, 0, 0)

