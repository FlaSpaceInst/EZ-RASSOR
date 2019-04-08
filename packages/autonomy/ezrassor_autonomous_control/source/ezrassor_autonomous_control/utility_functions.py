import rospy
import time
import nav_functions as nf

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

def self_check(world_state, ros_util):
    """ Check for unfavorable states in the system and handle or quit gracefully. """
    if ros_util.auto_function_command == 32:
        ros_util.status_pub.publish("Cancel Auto Function Command Recieved")
        ros_util.command_pub.publish(ros_util.commands['null'])
        ros_util.command_pub.publish(ros_util.commands['kill_bit'])
        return -1
    if world_state.state_flags['on_side'] == True:
        ros_util.status_pub.publish("On Side - Attempting Auto Self Right")
        return 2
    if world_state.state_flags['battery'] < 10:
        ros_util.status_pub.publish("Low Battery - Returning to Base")
        world_state.state_flags['target_location'] = [0,0]
        return 3
    if world_state.state_flags['hardware_status'] == False:
        ros_util.status_pub.publish("Hardware Failure Shutting Down")
        ros_util.command_pub.publish(ros_util.commands['null'])
        ros_util.command_pub.publish(ros_util.commands['kill_bit'])
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
    start_x = world_state.state_flags['positionX']
    start_y = world_state.state_flags['positionY']

    new_heading = (world_state.state_flags['heading'] + 45) % 360

    print(world_state.state_flags['heading'], new_heading) 

    while not ((new_heading - 5) < world_state.state_flags['heading'] < (new_heading + 5)):
        ros_util.command_pub.publish(ros_util.commands['left'])
        ros_util.rate.sleep()

    while nf.euclidean_distance(start_x, world_state.state_flags['positionX'], 
                                start_y, world_state.state_flags['positionY']) < 2:
        ros_util.command_pub.publish(ros_util.commands['forward'])
        ros_util.rate.sleep()


def dodge_right(world_state, ros_util):
    start_x = world_state.state_flags['positionX']
    start_y = world_state.state_flags['positionY']

    new_heading = (world_state.state_flags['heading'] - 45) % 360

    print(world_state.state_flags['heading'], new_heading) 

    while not ((new_heading - 5) < world_state.state_flags['heading'] < (new_heading + 5)):
        ros_util.command_pub.publish(ros_util.commands['right'])
        ros_util.rate.sleep()

    while nf.euclidean_distance(start_x, world_state.state_flags['positionX'], 
                                start_y, world_state.state_flags['positionY']) < 2:
        
        ros_util.command_pub.publish(ros_util.commands['forward'])
        ros_util.rate.sleep()

