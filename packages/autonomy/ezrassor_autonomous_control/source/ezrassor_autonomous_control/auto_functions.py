#!/usr/bin/env python

import rospy
import utility_functions as uf
import nav_functions as nf

def at_target(positionX, positionY, targetX, targetY, threshold):
    """ Determine if the current position is within
        the desired threshold of the target position.
    """
    value = ((targetX - threshold) < positionX < (targetX + threshold)
            and (targetY - threshold) < positionY < (targetY + threshold))

    return value

def auto_drive_location(world_state, ros_util):
    """ Navigate to location. Avoid obstacles while moving toward location. """
    rospy.loginfo('Auto-driving to [%s, %s]...',
                  str(world_state.target_location.x),
                  str(world_state.target_location.y))

    # Set arms up for travel
    uf.set_front_arm_angle(world_state, ros_util, 1.3)
    uf.set_back_arm_angle(world_state, ros_util, 1.3)

    # Check if robot should stpop moving
    if uf.self_check(world_state, ros_util) != 1:
        rospy.logdebug('Status check failed.')
        return

    # Before we head towards our goal, turn to face it.
    # Get new heading angle relative to current heading
    new_heading_degrees = nf.calculate_heading(world_state)
    angle2goal_radians = nf.adjust_angle(world_state.heading,
                                         new_heading_degrees)

    # If our angle is less than zero, then we would expect a right turn
    # otherwise turn left.
    if angle2goal_radians < 0:
        direction = 'right'
    else:
        direction = 'left'

    uf.turn(new_heading_degrees, direction, world_state, ros_util)
    ros_util.publish_actions('stop', 0, 0, 0, 0)

    # Main loop until location is reached
    while not at_target(world_state.positionX, world_state.positionY,
                        world_state.target_location.x,
                        world_state.target_location.y, ros_util.threshold):

        # Set arms up for travel
        uf.set_front_arm_angle(world_state, ros_util, 1.3)
        uf.set_back_arm_angle(world_state, ros_util, 1.3)

        if uf.self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        angle = uf.get_turn_angle(world_state, ros_util)

        # If our angle is less than zero, then we would expect a right turn
        # otherwise turn left.
        direction = 'right' if angle < 0 else 'left'

        # Turn towards the direction chosen.
        uf.turn(nf.rel_to_abs(world_state.heading, angle), direction,
                world_state, ros_util)

        # Move towards the direction chosen.
        uf.move(ros_util.move_increment, world_state, ros_util)

    rospy.loginfo('Destination reached!')
    ros_util.publish_actions('stop', 0, 0, 0, 0)

def auto_dig(world_state, ros_util, duration):
    """ Rotate both drums inward and drive forward
        for duration time in seconds.
    """
    rospy.loginfo('Auto-digging for %d seconds...', duration)

    uf.set_front_arm_angle(world_state, ros_util, -0.1)
    uf.set_back_arm_angle(world_state, ros_util, -0.1)

    # Perform Auto Dig for the desired Duration
    t = 0
    while t < duration*40:
        if uf.self_check(world_state, ros_util) != 1:
            return
        ros_util.publish_actions('forward', 0, 0, 1, 1)
        t+=1
        ros_util.rate.sleep()

    ros_util.publish_actions('stop', 0, 0, 0, 0)

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
    while t < duration*40:
        if uf.self_check(world_state, ros_util) != 1:
            return
        ros_util.publish_actions('stop', 0, 0, -1, -1)
        t+=1
        ros_util.rate.sleep()

    new_heading = world_state.heading = (world_state.heading + 180) % 360

    while not ((new_heading - 1) < world_state.heading < (new_heading + 1)):
            ros_util.publish_actions('left', 0, 0, 0, 0)
            ros_util.rate.sleep()

    while t < duration*30:
        ros_util.publish_actions('stop', 0, 0, -1, -1)
        t+=1
        ros_util.rate.sleep()

    ros_util.publish_actions('stop', 0, 0, 0, 0)
