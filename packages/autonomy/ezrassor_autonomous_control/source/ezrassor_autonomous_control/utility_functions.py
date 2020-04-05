import rospy
import nav_functions as nf
import math
from geometry_msgs.msg import Twist

scan = None

def on_scan_update(new_scan):
    global scan
    scan = new_scan

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

""" Turns the robot to the given heading

Given a heading and a direction to turn to, this function turns the robot from
our current heading to the new heading. A ramping function (using the sine
function from 0 to pi) is used.
"""
def turn(new_heading, direction, world_state, ros_util):
    # Calculate how many degrees that we need to turn.
    angle_dist = abs((new_heading - world_state.heading + 180) % 360 - 180)
    angle_traveled = 0

    # Turn the number of degrees towards your new heading.
    while angle_traveled < angle_dist - 2:
        if self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        old_heading = world_state.heading

        # Maps angle traveled to sin(x) function for velocity ramping.
        # X is bounded between 0 and angle_dist
        # Y is bounded between 0 and max_angular_velocity
        turn_velocity = (ros_util.max_angular_velocity *
                         math.sin((angle_traveled/math.pi) * (10/angle_dist)))

        # Cap our minimum velocity at 1/10 our max velocity.
        turn_velocity = max(turn_velocity, ros_util.max_angular_velocity / 10)

        if direction is 'right':
            turn_velocity *= -1

        twist_message = Twist()
        twist_message.angular.z = turn_velocity
        ros_util.movement_pub.publish(twist_message)

        ros_util.rate.sleep()

        angle_traveled += abs((world_state.heading-old_heading+180) % 360 - 180)

""" Moves the robot a given distance or until an obstacle is encountered

Given a distance and a direction to move towards, this function moves the robot
until it has moved that distance or it encounters an obstacle. A ramping
function (using the sine function from 0 to pi) is used.
"""
def move(dist, world_state, ros_util, direction='forward'):
    # Get current distance from EZ-RASSOR to our current best target location.
    old_x = world_state.positionX
    old_y = world_state.positionY
    dist_traveled = 0
    dist_to_goal = math.sqrt((world_state.target_location.x - old_x) **2 +
                             (world_state.target_location.y - old_y) **2)

    # Either move dist amount or the distance to the goal, whichever is smaller.
    move_dist = min(dist, dist_to_goal)

    # Drive the designated distance and exit when we pass it or there is an
    # obstacle in the way.
    while dist_traveled < move_dist:
        if self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        # Exit if we come too close to an obstacle
        if not nf.angle_is_safe(0, ros_util.obstacle_threshold / 2.0,
                                ros_util.obstacle_buffer, scan,
                                ros_util.obstacle_threshold):
            rospy.loginfo("Obstacle too close! Stopping!")
            ros_util.publish_actions("stop", 0, 0, 0, 0)
            break

        # Maps distance traveled to sin(x) function for velocity ramping.
        # X is bounded between 0 and move_dist
        # Y is bounded between 0 and max_linear_velocity
        move_velocity = (ros_util.max_linear_velocity *
                         math.sin((dist_traveled/math.pi) * (10/move_dist)))

        # Cap our minimum velocity at 1/10 our max velocity.
        move_velocity = max(move_velocity, ros_util.max_linear_velocity / 10)

        if direction is 'backward':
            move_velocity *= -1

        twist_message = Twist()
        twist_message.linear.x = move_velocity
        ros_util.movement_pub.publish(twist_message)

        ros_util.rate.sleep()

        dist_traveled = math.sqrt((world_state.positionX - old_x) ** 2 +
                                  (world_state.positionY - old_y) ** 2)

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

""" Returns the best direction to go towards to get to the goal

This function returns the direction the robot should move towards. If no safe
angle is found, the WedgeBug algorithm is used to look for a safe angle in an
adjacent view. Return once a safe angle is found.
"""
def get_turn_angle(world_state, ros_util):
    # Iterate over all of the laser beams in our current scan and determine the
    # best angle to turn towards.
    best_angle = nf.get_best_angle(world_state, ros_util.obstacle_buffer, scan,
                                   ros_util.obstacle_threshold)

    # Loop until we find a safe angle.
    while True:
        # If the best angle is None, we need to look at an adjacent wedge that
        # we have not already checked
        if best_angle is None:
            # These variables are used to oscillate between wedges if we enter
            # visual boundary following mode below.
            switchDirection = -1
            wedgeDist = 0
            wedgeSize = (scan.angle_max - scan.angle_min) / 2.0
            rospy.loginfo("There is nowhere to go in the current wedge. " +
                          "Turning to an adjacent wedge.")

            # Keep checking adjacent wedges until we find a safe angle.
            while best_angle is None:
                set_front_arm_angle(world_state, ros_util, 1.3)
                set_back_arm_angle(world_state, ros_util, 1.3)

                switchDirection *= -1
                wedgeDist += 1

                if switchDirection < 0:
                    direction = 'left'
                else:
                    direction = 'right'

                # Turn to an adjacent wedge and check if we can see some way to
                # progress towards the goal.
                turn(nf.rel_to_abs(world_state.heading, wedgeSize * wedgeDist),
                     direction, world_state, ros_util)
                ros_util.publish_actions('stop', 0, 0, 0, 0)
                ros_util.rate.sleep()
                rospy.sleep(0.1)

                rospy.loginfo("Currently at wedge W{}".format(wedgeDist - 1))
                best_angle = nf.get_best_angle(world_state,
                                               ros_util.obstacle_buffer, scan,
                                               ros_util.obstacle_threshold)

        # If we over-turned when doing the wedge, try turning back more towards
        # the goal until we see an obstacle in our view.
        wedgeSize = (scan.angle_max - scan.angle_min) / 20.0
        buffer_angle = math.atan(ros_util.obstacle_buffer /
                                 ros_util.obstacle_threshold)
        min_angle = scan.angle_min + buffer_angle
        max_angle = scan.angle_max - buffer_angle
        best_index = int((best_angle - scan.angle_min) / scan.angle_increment)
        min_index = int((min_angle - scan.angle_min) / scan.angle_increment)
        max_index = int((max_angle - scan.angle_min) / scan.angle_increment)

        # If the best angle we can go to is the left or right edge of the
        # LaserScan, that means we can probably turn more in that direction.
        while best_index <= min_index or best_index >= max_index:
            # Get direction that would bring us closer to facing the goal.
            if best_angle < 0:
                direction = 'right'
            else:
                direction = 'left'

            # Turn slightly in that direction and check if it's safe to turn
            # more towards the goal.
            turn(nf.rel_to_abs(world_state.heading, wedgeSize), direction,
                 world_state, ros_util)
            ros_util.publish_actions('stop', 0, 0, 0, 0)
            ros_util.rate.sleep()
            rospy.sleep(0.1)
            best_angle = nf.get_best_angle(world_state,
                                           ros_util.obstacle_buffer, scan,
                                           ros_util.obstacle_threshold)

            # If we turned too far back and now have no safe angle to go to,
            # break and restart the wedge algorithm.
            if best_angle is None:
                break

            # Get LaserScan index of best angle
            best_index = int((best_angle-scan.angle_min) / scan.angle_increment)

        # Exit once we've found a safe angle.
        if best_angle is not None:
            return best_angle
