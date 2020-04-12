import rospy
import math
import utility_functions as uf
import nav_functions as nf
import arm_force as armf


def at_target(world_state, ros_util):
    """ Determine if the current position is within 
        the desired threshold of the target position. 
    """
    positionX = world_state.positionX
    positionY = world_state.positionY

    targetX = world_state.target_location.x
    targetY = world_state.target_location.y

    value = ((targetX - ros_util.threshold) < positionX < (targetX + ros_util.threshold)
             and (targetY - ros_util.threshold) < positionY < (targetY + ros_util.threshold))

    return not value

def charge_battery(world_state, ros_util, waypoint_server=None) :
    """ Charge the rover's battery for a designated duration until battery is 100% """

    # world_state.battery = 100
    ros_util.publish_actions('stop', 0, 0, 0, 0)
    feedback = uf.send_feedback(world_state, waypoint_server)
    while world_state.battery < 100 :
        rospy.sleep(0.1)
        world_state.battery += 3
        # rospy.loginfo(world_state.battery)
    world_state.battery = 100    
    feedback = uf.send_feedback(world_state, waypoint_server)

def auto_drive_location(world_state, ros_util, waypoint_server=None):
    """ Navigate to location. Avoid obstacles while moving toward location. """

    # Action server will print it's own info
    if waypoint_server is None:
        rospy.loginfo('Auto-driving to [%s, %s]...',
                  str(world_state.target_location.x),
                  str(world_state.target_location.y))

    # Send feedback to waypoint client if being controlled by swarm controller
    feedback = uf.send_feedback(world_state, waypoint_server)

    # Set arms up for travel
    uf.set_front_arm_angle(world_state, ros_util, 1.3)
    uf.set_back_arm_angle(world_state, ros_util, 1.3)

    # Main loop until location is reached
    while at_target(world_state, ros_util):

        # Check rover battery, hardware, and if it's flipped over
        if uf.self_check(world_state, ros_util) != 1:

            # Cancel action server request is self check failed
            if waypoint_server is not None:
                waypoint_server.set_preempted()

            rospy.logdebug('Status check failed.')
            return

        # Get new heading angle relative to current heading as (0,0)
        new_heading = nf.calculate_heading(world_state, ros_util)
        angle_difference = nf.adjust_angle(world_state.heading, new_heading)

        if angle_difference < 0:
            direction = 'right'
        else:
            direction = 'left'

        # Adjust heading until it matches new heading
        while not ((new_heading - 5) < world_state.heading < (new_heading + 5)):
            ros_util.publish_actions(direction, 0, 0, 0, 0)
            ros_util.rate.sleep()

        # Avoid obstacles by turning left or right if warning flag is raised
        if world_state.warning_flag == 1:
            uf.dodge_right(world_state, ros_util)
        if world_state.warning_flag == 2:
            uf.dodge_left(world_state, ros_util)
        if world_state.warning_flag == 3:
            uf.reverse_turn(world_state, ros_util)
            rospy.loginfo('Avoiding detected obstacle...')

        # Otherwise go forward
        ros_util.publish_actions('forward', 0, 0, 0, 0)
        ros_util.rate.sleep()

        world_state.battery -= (0.00775/10)

        # Send feedback to waypoint action client
        feedback = uf.send_feedback(world_state, waypoint_server)

    # Action server will print it's own info
    if waypoint_server is None:
        rospy.loginfo('Destination reached!')

    ros_util.publish_actions('stop', 0, 0, 0, 0)

    # return feedback


def auto_dig(world_state, ros_util, duration, waypoint_server=None) :
    """ Rotate both drums inward and drive forward 
        for duration time in seconds. 
    """
    feedback = uf.send_feedback(world_state, waypoint_server)
    rospy.loginfo('Auto-digging for %d seconds...', duration)

    uf.set_front_arm_angle(world_state, ros_util, -0.1)
    uf.set_back_arm_angle(world_state, ros_util, -0.1)

    # Perform Auto Dig for the desired Duration
    t = 0
    
    while t < duration:
        if uf.self_check(world_state, ros_util) != 1:
            return
        # Dig while moving forward for 5 seconds
        if world_state.battery <= 30 :
            break
        ros_util.publish_actions('forward', 0, 0, 1, 1)
        feedback = uf.send_feedback(world_state, waypoint_server)
        t += 5
        rospy.sleep(5)
        world_state.battery -= 20
        # Dig while moving backward for 5 seconds
        ros_util.publish_actions('reverse', 0, 0, 1, 1)
        feedback = uf.send_feedback(world_state, waypoint_server)
        t += 5
        rospy.sleep(5)

    ros_util.publish_actions('stop', 0, 0, 0, 0)
    feedback = uf.send_feedback(world_state, waypoint_server)
    rospy.loginfo('Done digging')

def auto_dock(world_state, ros_util):
    """ Dock with the hopper. """

    rospy.loginfo('Auto-returning to origin...')

    old_target_x = world_state.target_location.x
    old_target_y = world_state.target_location.y

    ros_util.threshold = 3

    world_state.target_location.x = 0
    world_state.target_location.y = 0

    auto_drive_location(world_state, ros_util)
    ros_util.threshold = .5

    world_state.target_location.x = old_target_x
    world_state.target_location.y = old_target_y


def auto_dump(world_state, ros_util, duration):
    """ Rotate both drums inward and drive forward 
        for duration time in seconds. 
    """
    rospy.loginfo('Auto-dumping drum contents...')

    uf.set_front_arm_angle(world_state, ros_util, 1.3)
    uf.set_back_arm_angle(world_state, ros_util, 1.3)

    t = 0
    while t < duration * 40:
        if uf.self_check(world_state, ros_util) != 1:
            return
        ros_util.publish_actions('stop', 0, 0, -1, -1)
        t += 1
        ros_util.rate.sleep()

    new_heading = world_state.heading = (world_state.heading + 180) % 360

    while not ((new_heading - 1) < world_state.heading < (new_heading + 1)):
        ros_util.publish_actions('left', 0, 0, 0, 0)
        ros_util.rate.sleep()

    while t < duration * 30:
        ros_util.publish_actions('stop', 0, 0, -1, -1)
        t += 1
        ros_util.rate.sleep()

    ros_util.publish_actions('stop', 0, 0, 0, 0)
