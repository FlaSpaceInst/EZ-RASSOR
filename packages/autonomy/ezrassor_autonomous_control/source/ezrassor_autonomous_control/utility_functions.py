import rospy
import time
import nav_functions as nf

def set_front_arm_angle(world_state, ros_util, target_angle):
    """ Set front arm to absolute angle target_angle in radians. """

    ros_util.status_pub.publish(
        "Setting Front Arm Angle to {} Radians".format(
            target_angle,
        )
    )

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

    ros_util.status_pub.publish(
        "Setting Back Arm Angle to {} Radians".format(
            target_angle,
        ),
    )

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
    
    if ros_util.auto_function_command == 32:
        ros_util.status_pub.publish("Cancel Auto Function Command Recieved")
        ros_util.publish_actions('stop', 0, 0, 0, 0)
        ros_util.control_pub.publish(False)
        return -1
    if world_state.on_side == True:
        ros_util.status_pub.publish("On Side - Attempting Auto Self Right")
        return 2
    if world_state.battery < 10:
        ros_util.status_pub.publish("Low Battery - Returning to Base")
        world_state.target_location = [0,0]
        return 3
    if world_state.hardware_status == False:
        ros_util.status_pub.publish("Hardware Failure Shutting Down")
        ros_util.publish_actions('stop', 1, 0, 0, 0)
        ros_util.control_pub.publish(False)
        return -1
    else:
        ros_util.status_pub.publish("Passed Status Check")
        return 1
        
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

    while world_state.warning_flag != 0:
        ros_util.publish_actions('left', 0, 0, 0, 0)
        ros_util.rate.sleep()

    while nf.euclidean_distance(start_x, world_state.positionX, 
                                start_y, world_state.positionY) < 2:
        ros_util.publish_actions('forward', 0, 0, 0, 0)
        ros_util.rate.sleep()


def dodge_right(world_state, ros_util):
    start_x = world_state.positionX
    start_y = world_state.positionY

    while world_state.warning_flag != 0:
        ros_util.publish_actions('right', 0, 0, 0, 0)
        ros_util.rate.sleep()

    while nf.euclidean_distance(start_x, world_state.positionX, 
                                start_y, world_state.positionY) < 2:
        
        ros_util.publish_actions('forward', 0, 0, 0, 0)
        ros_util.rate.sleep()


def self_right_from_side(world_state, ros_util):
    """ Flip EZ-RASSOR over from its side. """

    self_right_execution_time = 0.5
    start_time = time.time()
    ros_util.status_pub.publish("Initiating Self Right")
    while(time.time() - start_time < self_right_execution_time):
        if world_state.on_side == False:
            ros_util.publish_actions('stop', 0, 0, 0, 0)
            return
        ros_util.publish_actions('stop', 0, 1, 0, 0)
        ros_util.publish_actions('stop', 1, 0, 0, 0)
            
    ros_util.publish_actions('stop', 0, 0, 0, 0)