import rospy
import math
import utility_functions as uf
import nav_functions as nf
import numpy as np

scan = None
threshold = 4.0
buffer = 1.5
move_dist = 3.0

def on_scan_update(new_scan):
    global scan
    scan = new_scan

def at_target(positionX, positionY, targetX, targetY, threshold):
    """ Determine if the current position is within 
        the desired threshold of the target position. 
    """
    value = ((targetX - threshold) < positionX < (targetX + threshold)
            and (targetY - threshold) < positionY < (targetY + threshold))

    return value

def auto_drive_location(world_state, ros_util):
    """ Navigate to location. Avoid obstacles while moving toward location. """
    rospy.loginfo('Auto-driving to [%s, %s]...',str(world_state.target_location.x),str(world_state.target_location.y))
    
    # Set arms up for travel
    uf.set_front_arm_angle(world_state, ros_util, 1.3)
    uf.set_back_arm_angle(world_state, ros_util, 1.3)

    # Before we head towards our goal, turn to face it.
    # Get new heading angle relative to current heading
    new_heading_degrees = nf.calculate_heading(world_state)
    angle2goal_radians = nf.adjust_angle(world_state.heading, new_heading_degrees)

    # If our angle is less than zero, then we would expect a right turn otherwise turn left.
    if angle2goal_radians < 0:
        direction = 'right'
    else:
        direction = 'left'

    uf.turn(new_heading_degrees, direction, world_state, ros_util)
    ros_util.publish_actions('stop', 0, 0, 0, 0)

    # Main loop until location is reached
    while not at_target(world_state.positionX, world_state.positionY, world_state.target_location.x, world_state.target_location.y, ros_util.threshold):

        rospy.loginfo("Starting nav loop!")

        if uf.self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        # Iterate over all of the laser beams in our scan wedge and determine the best angle to turn and x,y point.
        best_angle = get_best_angle(world_state)

        while True:
            # If the best angle is None, we need to look at an adjacent wedge that we have not already seen
            if best_angle is None:

                # These variables are used to oscillate between wedges if we enter visual boundary following mode below.
                switchDirection = -1
                wedgeDist = 0
                wedgeSize = (scan.angle_max - scan.angle_min)
                rospy.loginfo("There is nowhere to go in the current wedge. Turning to an adjacent wedge.")

                while best_angle is None:

                    switchDirection *= -1
                    wedgeDist += 1

                    if switchDirection < 0:
                        direction = 'left'
                    else:
                        direction = 'right'

                    # Turn to an adjacent wedge and check if we can see some way to progress towards the goal.
                    uf.turn(rel_to_abs(world_state.heading, (wedgeSize) * wedgeDist), direction, world_state, ros_util)
                    ros_util.publish_actions('stop', 0, 0, 0, 0)
                    ros_util.rate.sleep()
                    rospy.sleep(0.1)

                    rospy.loginfo("Currently at wedge W{}".format(wedgeDist - 1))
                    best_angle = get_best_angle(world_state)
                
            wedgeSize = (scan.angle_max - scan.angle_min) / 10

            buffer_angle = math.atan(buffer / threshold)
            min_angle = scan.angle_min + buffer_angle
            max_angle = scan.angle_max - buffer_angle

            best_index = int((best_angle - scan.angle_min) / scan.angle_increment)
            min_index = int((min_angle - scan.angle_min) / scan.angle_increment)
            max_index = int((max_angle - scan.angle_min) / scan.angle_increment)
            rospy.loginfo("best index: {}, min index: {}, max index: {}".format(best_index, min_index, max_index))

            while best_index <= min_index or best_index >= max_index:
                rospy.loginfo("Turning more because we can")
                if best_angle < 0:
                    direction = 'right'
                else:
                    direction = 'left'
                uf.turn(rel_to_abs(world_state.heading, wedgeSize), direction, world_state, ros_util)
                ros_util.publish_actions('stop', 0, 0, 0, 0)
                ros_util.rate.sleep()
                rospy.sleep(0.1)
                best_angle = get_best_angle(world_state)
                if best_angle is None:
                    break
                best_index = int((best_angle - scan.angle_min) / scan.angle_increment)

            if best_angle is not None:
                break

        # If our angle is less than zero, then we would expect a right turn otherwise turn left.
        if best_angle < 0:
            direction = 'right'
        else:
            direction = 'left'

        # Turn towards the direction chosen.
        uf.turn(rel_to_abs(world_state.heading, best_angle), direction, world_state, ros_util)
        ros_util.publish_actions('stop', 0, 0, 0, 0)
        ros_util.rate.sleep()
        rospy.sleep(0.1)

        # This is the current distance from EZ-RASSOR to our current best target location.
        old_x = world_state.positionX
        old_y = world_state.positionY
        distance_traveled = 0
        distance_to_goal = math.sqrt((world_state.target_location.x - old_x) **2 + (world_state.target_location.y - old_y) **2)

        #Move either the move_dist or the distance to the goal (if we are too close).
        # Drive towards the best point while there is no obstacle in your way.
        while distance_traveled < min(move_dist, distance_to_goal):

            if uf.self_check(world_state, ros_util) != 1:
                rospy.logdebug('Status check failed.')
                return

            # If obstacles are too close, figure out how to get around them.
            if not angle_is_safe(0, threshold):
                rospy.loginfo("Obstacle too close! Stopping!")
                ros_util.publish_actions("stop", 0, 0, 0, 0)
                break

            ros_util.publish_actions('forward', 0, 0, 0, 0)
            ros_util.rate.sleep()

            distance_traveled = math.sqrt((world_state.positionX - old_x) ** 2 + (world_state.positionY - old_y) ** 2)

        rospy.loginfo('Reached edge of obstacle')

        ros_util.rate.sleep()

    rospy.loginfo('Destination reached!')
    ros_util.publish_actions('stop', 0, 0, 0, 0)

# Check if EZ-RASSOR can fit through a space given the angle to turn towards it.
# Angle is in radians and dist is the distance to an object or threshold value passed in.
def angle_is_safe(angle, dist):
    # Calculate how much to change angle in order for robot to clear obstacle.
    buffer_angle = math.atan(buffer / dist)

    # These are in radians.
    angle1 = angle - buffer_angle
    angle2 = angle + buffer_angle

    # These are the indices in our scan.ranges array that correspond to the angles above.
    index1 = int((angle1 - scan.angle_min) / scan.angle_increment)
    index2 = int((angle2 - scan.angle_min) / scan.angle_increment)

    # These indices allow us to check if EZ-RASSOR can fit through the given area.
    start = min(index1, index2)
    end = max(index1, index2)

    # If we are looking outside of our current wedge, attempt to turn towards the unseen area and return if it is safe.
    if start < 0 or end >= len(scan.ranges):
        return False

    # Check to see if something is blocking where we are trying to fit through.
    for i in range(start, end):
        if (not np.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
            return False

    return True

# Iterate through the laser scan in our current view (wedge) and determine our best move towards the goal.
def get_best_angle(world_state):

    best_score = None
    best_angle = None

    rospy.loginfo("currently in the get best angle function.")
    # rospy.loginfo(scan.ranges)

    for i in range(0, len(scan.ranges)):
        angle = scan.angle_min + i * scan.angle_increment

        # We only enter this code if we have found a discontinuity or obstacle edge that we cannot use.
        if not angle_is_safe(angle, threshold):
            continue

        new_heading_degrees = nf.calculate_heading(world_state)
        angle2goal_radians = nf.adjust_angle(world_state.heading, new_heading_degrees)

        score = abs(angle2goal_radians - angle)

        if best_score is None or score < best_score:
            best_score = score
            best_angle = angle

    return best_angle

def rel_to_abs(current_heading_degrees, relative_heading_radians):
    relative_heading_degrees = (180 * relative_heading_radians / math.pi)
    abs_heading = relative_heading_degrees + current_heading_degrees

    if abs_heading < 0:
        abs_heading += 360

    return abs_heading

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
