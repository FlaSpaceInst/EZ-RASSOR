import rospy
import math
import utility_functions as uf
import nav_functions as nf
import arm_force as armf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import copy
import numpy as np

scan = None
threshold = 4.0

# threshold that determines if a point in the laser scan is a discontinuity
dist_thresh = 3.0
buffer_safe = 1.3
buffer = buffer_safe * 1.2

# TODO: FIX THIS!! This is why we have nan values at the first and last index's in our scan.ranges array.
def on_scan_update(new_scan):
    ranges = [float("nan")] * len(new_scan.ranges)

    for i in range(1, len(new_scan.ranges)-1):
        if math.isnan(new_scan.ranges[i]) and not math.isnan(new_scan.ranges[i-1]) and not math.isnan(new_scan.ranges[i+1]):
            ranges[i] = (new_scan.ranges[i-1]+new_scan.ranges[i+1])/2
        else:
            ranges[i] = new_scan.ranges[i]

    new_scan.ranges = ranges
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
    
    # Main loop until location is reached
    while not at_target(world_state.positionX, world_state.positionY, world_state.target_location.x,world_state.target_location.y, ros_util.threshold):

        # TODO: In any loop that we move or turn, we need to make sure that we dont hit an obstacle!!!!!
        rospy.loginfo("Starting nav loop!")

        if uf.self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        # Get new heading angle relative to current heading
        new_heading_degrees = nf.calculate_heading(world_state)
        angle2goal_radians = nf.adjust_angle(world_state.heading, new_heading_degrees)

        # If our angle is less than zero, then we would expect a right turn otherwise turn left.
        if angle2goal_radians < 0:
            direction = 'right'
        else:
            direction = 'left'

        # If we are not facing the goal, turn to face it.
        if angle2goal_radians > scan.angle_max or angle2goal_radians < scan.angle_min:
            rospy.loginfo("Turning towards the goal since we are not facing it!")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            ros_util.publish_actions('stop', 0, 0, 0, 0)
            continue

        # If we can see the goal (no obstacles in the way), go towards it
        if angle_is_safe(angle2goal_radians, threshold, world_state, ros_util, 0):
            rospy.loginfo("Turning towards the goal and there is no obstacle in our way!")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            ros_util.publish_actions('forward', 0, 0, 0, 0)
            ros_util.rate.sleep()

        # If we enter here, there is an obstacle blocking our path. Use wedgebug to avoid obstacles.
        else:
            # Iterate over all of the laser beams in our scan wedge and determine the best angle to turn and x,y point.
            best_angle, best_x, best_y = get_best_angle(world_state, ros_util)

            #TODO: If the best angle is None, we need to look at an adjacent wedge that we have not already seen!
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

                    rospy.loginfo("Currently at wedge W{}".format(wedgeDist - 1))
                    best_angle, best_x, best_y = get_best_angle(world_state, ros_util)

            # If our angle is less than zero, then we would expect a right turn otherwise turn left.
            if best_angle < 0:
                direction = 'right'
            else:
                direction = 'left'

            # Turn towards the direction chosen.
            uf.turn(rel_to_abs(world_state.heading, best_angle), direction, world_state, ros_util)

            # This is the current distance from EZ-RASSOR to our current best target location.
            distance_to_best = math.sqrt((world_state.positionX - best_x) ** 2 + (world_state.positionY - best_y) ** 2)
            old_x = world_state.positionX
            old_y = world_state.positionY

            # Drive towards the best point while there is no obstacle in your way.
            while not at_target(world_state.positionX, world_state.positionY, best_x, best_y, 0.3):

                if uf.self_check(world_state, ros_util) != 1:
                    rospy.logdebug('Status check failed.')
                    return

                # TODO: If EZ-RASSOR meets a parobolic object, it will continue to go back and forth! Use boundary following.
                # TODO: Make sure that obstacles that are at an angle to use a greater threshold.
                # If obstacles are too close, figure out how to get around them.
                if np.nanmin(scan.ranges) <= 1.1:
                    rospy.loginfo("Obstacle too close! Stopping!")
                    ros_util.publish_actions("stop", 0, 0, 0, 0)
                    break

                ros_util.publish_actions('forward', 0, 0, 0, 0)
                ros_util.rate.sleep()

                distance_traveled = math.sqrt((world_state.positionX - old_x) ** 2 + (world_state.positionY - old_y) ** 2)

                # If we passed our current target location, stop and continue our path planning.
                if distance_traveled >= distance_to_best:
                    rospy.loginfo("We passed our current best (x,y). Breaking out of loop!")
                    break

            rospy.loginfo('Reached edge of obstacle')
            
        ros_util.rate.sleep()

    rospy.loginfo('Destination reached!')
    ros_util.publish_actions('stop', 0, 0, 0, 0)

# Check if EZ-RASSOR can fit through a space given the angle to turn towards it.
# Angle is in radians and dist is the distance to an object or threshold value passed in.
def angle_is_safe(angle, dist, world_state, ros_util, recursive_flag):

    # If everything in front of us is open, return true as this is safe to traverse.
    if np.isnan(np.nanmin(scan.ranges)):
        return True

    # Calculate how much to change angle in order for robot to clear obstacle.
    buffer_angle = math.atan(buffer_safe / dist)

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

        # If we have turned towards an unknown area and still cannot see it in our wedge, return false.
        if recursive_flag > 0:
            return False

        # Check where we cant see and prepare to turn towards it. If it is blocked, return false.
        if start < 0:
            for i in range(0, end):
                if (not np.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
                    rospy.loginfo("We cannot fit in the space seen in the given wedge! Return False.")
                    return False

            direction = 'right'
            directionBack = 'left'
        else:
            for i in range(start, len(scan.ranges) - 1):
                if (not np.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
                    rospy.loginfo("We cannot fit in the space seen in the given wedge! Return False.")
                    return False

            direction = 'left'
            directionBack = 'right'

        rospy.loginfo("We are out of the scan.ranges array! Turning towards the area of uncertainty.")
        # This is where we are facing before we turn.
        old_heading = world_state.heading

        # Turn towards the area of uncertainty and check if it is open and then turn back.
        uf.turn(rel_to_abs(world_state.heading, angle), direction, world_state, ros_util)
        retVal = angle_is_safe(angle, dist, world_state, ros_util, recursive_flag + 1)
        uf.turn(old_heading, directionBack, world_state, ros_util)
        return retVal

    # Check to see if something is blocking where we are trying to fit through.
    for i in range(start, end):
        if (not np.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
            rospy.loginfo("We cannot fit in the space seen in the given wedge! Return False.")
            return False

    rospy.loginfo("This is a good angle and is safe. Take it!")
    return True

# Iterate through the laser scan in our current view (wedge) and determine our best move towards the goal.
# TODO: The min and max values of our scan.ranges array are always nan even when there is nothing there.
def get_best_angle(world_state, ros_util):

    best_total_dist = None
    best_angle = None
    best_x = None
    best_y = None

    rospy.loginfo("currently in the get best angle function.")
    rospy.loginfo(scan.ranges)

    for i in range(2, len(scan.ranges) - 1): # TODO: Should we ignore these values? Looping from 2 to (scan.ranges - 1)
        # Get range for current and previous LaserScan angles.
        current = scan.ranges[i]
        previous = scan.ranges[i-1]

        # Both current and previous are NaN: no obstacle edge.
        #TODO: Added check that we only want to look at discontinuities that are within our threshold distance to eliminate excessive turning/checking.
        # if (math.isnan(current) or current > threshold) and (math.isnan(previous) or previous > threshold):
        #     continue
        if math.isnan(current) and math.isnan(previous):
            continue
        # Only one of current and previous are NaN: obstacle edge.
        elif math.isnan(current) or math.isnan(previous):
            if math.isnan(current):
                obst_to_safe = True
                dist = previous
                idx = i
            else:
                obst_to_safe = False
                dist = current
                idx = i-1
        # Neither are NaN: there must be sufficient difference for obstacle edge.
        elif abs(current - previous) > dist_thresh:
            if current > previous:
                obst_to_safe = True
                dist = previous
                idx = i
            else:
                obst_to_safe = False
                dist = current
                idx = i-1
        # No obstacle edge in any other case.
        else:
            continue

        rospy.loginfo("Discontinuity found at {}, {}".format(previous,current))

        # If we have made it to this point, we have found an obstacle edge and will now check if it is sage to traverse.
        if math.isnan(dist):
            rospy.loginfo("Dist is nan. Setting dist to our current threshold value.")
            dist = threshold

        # Calculate angle at this index.
        d_angle = scan.angle_min + idx * scan.angle_increment

        # Calculate how much to change angle in order for robot to clear obstacle.
        buffer_angle = math.atan(buffer / dist)

        # Add room for clearing obstacle to make sure we can fit.
        if obst_to_safe:
            d_angle += buffer_angle
        else:
            d_angle -= buffer_angle

        # We only enter this code if we have found a discontinuity or obstacle edge that we cannot use.
        if not angle_is_safe(d_angle, dist, world_state, ros_util, 0):  # TODO: Should this be dist or a slightly greater value????
            continue

        # Add current heading to get angle in world reference.
        total_angle = (world_state.heading * math.pi / 180) + d_angle

        # Calculate the change in X, Y to get to the edge of the obstacle.
        d_x = dist * math.cos(total_angle)
        d_y = dist * math.sin(total_angle)

        # Calculate coordinates of the edge of the obstacle.
        obst_x = world_state.positionX + d_x
        obst_y = world_state.positionY + d_y

        # Calculate heuristic: distance between the goal and the edge of the obstacle. We want to minimize this!
        heur_dist = math.sqrt((world_state.target_location.x - obst_x) ** 2 + (world_state.target_location.y - obst_y) ** 2)

        # Save direction that minimizes veering off-track from our goal while avoiding obstacles.
        if best_total_dist is None or (heur_dist + dist) < best_total_dist:
            best_total_dist = heur_dist + dist
            best_angle = d_angle
            best_x = obst_x
            best_y = obst_y
            # TODO: Added this optimization to reduce turning.
            break

    return best_angle, best_x, best_y

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