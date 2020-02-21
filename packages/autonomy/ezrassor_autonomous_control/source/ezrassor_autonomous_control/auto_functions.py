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
buffer_safe = 1.1
buffer = buffer_safe * 1.2

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
    rospy.loginfo('Auto-driving to [%s, %s]...',
                  str(world_state.target_location.x),
                  str(world_state.target_location.y))
    
    # Set arms up for travel
    uf.set_front_arm_angle(world_state, ros_util, 1.3)
    uf.set_back_arm_angle(world_state, ros_util, 1.3)
    
    # Main loop until location is reached
    while not at_target(world_state.positionX, world_state.positionY, world_state.target_location.x,
                        world_state.target_location.y, ros_util.threshold):

        # TODO: In any loop that we move or turn, we need to make sure that we dont hit an obstacle!!!!!

        rospy.loginfo("Starting nav loop!")

        if uf.self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        # Get new heading angle relative to current heading
        new_heading_degrees = nf.calculate_heading(world_state, ros_util)
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
            continue

        scan_index = int((angle2goal_radians - scan.angle_min) / scan.angle_increment)

        rospy.loginfo("np.isnan(np.nanmin(scan.ranges)) = {}".format(np.isnan(np.nanmin(scan.ranges))))

        # If we can see the goal (no obstacles in the way), go towards it
        if angle_is_safe(angle2goal_radians, threshold, world_state, ros_util):
            rospy.loginfo("Turning towards the goal and there is no obstacle in our way!")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            ros_util.publish_actions('forward', 0, 0, 0, 0)
            ros_util.rate.sleep()

        # If we enter here, there is an obstacle blocking our path. Use wedgebug to avoid obstacles.
        else:
            # This is what we will be using to oscillate back and forth between wedge directions.
            switchDirection = -1
            wedgeDist = 0

            rospy.loginfo(scan.ranges)

            best_angle, best_x, best_y = get_best_angle(world_state, ros_util)

            rospy.loginfo("ranges max = {}, ranges min = {}, Difference between angles = {}".format(scan.angle_max,scan.angle_min, (scan.angle_max - scan.angle_min)))

            # TODO: If the best angle is None, we need to look at an adjacent wedge that we have not already seen!
            if best_angle is None:

                continue
                # # Continue to check different wedges in search for a possible way to progress.
                # while best_angle is None:   #TODO: Dont just take the first best angle that we see. We want to always pick the one that will take us closest to the goal!
                #     switchDirection *= -1
                #     wedgeDist += 1
                #     rospy.loginfo("Nowhere to go in current wedge. Checking different wedge! At wedge W{}".format(wedgeDist-1))
                #
                #     if switchDirection < 0:
                #         direction = 'left'
                #     else:
                #         direction = 'right'
                #
                #     new_angle = ((scan.angle_max - scan.angle_min) / 6) * wedgeDist
                #     rospy.loginfo("Turning {} to the {}.".format(new_angle * 180 / math.pi, direction))
                #     # Turn towards the direction chosen.
                #     uf.turn(rel_to_abs(world_state.heading, new_angle), direction, world_state, ros_util)
                #     rospy.loginfo("scan time = {}".format(scan.header.stamp))
                #     best_angle, best_x, best_y = get_best_angle(world_state, ros_util)

            if best_angle < 0:
                direction = 'right'
            else:
                direction = 'left'

            # Turn towards the direction chosen.
            uf.turn(rel_to_abs(world_state.heading, best_angle), direction, world_state, ros_util)

            # This is the current distance from EZ-RASSOR to our current destination.
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
                # TODO: Deadlocks here when too close to an obstacle!!!

                # If obstacles are too close, figure out how to get around them.
                if np.nanmin(scan.ranges) <= 0.8:
                    rospy.loginfo("Obstacle too close! Stopping!")
                    ros_util.publish_actions("stop", 0, 0, 0, 0)
                    break

                ros_util.publish_actions('forward', 0, 0, 0, 0)
                ros_util.rate.sleep()

                distance_traveled = math.sqrt((world_state.positionX - old_x) ** 2 + (world_state.positionY - old_y) ** 2)

                # If we passed our current (x,y) objective break and restart our path planning.
                if distance_traveled >= distance_to_best:
                    rospy.loginfo("We passed our current best (x,y). Breaking out of loop!")
                    break

            rospy.loginfo('Reached edge of obstacle')
            
        ros_util.rate.sleep()

    rospy.loginfo('Destination reached!')
    ros_util.publish_actions('stop', 0, 0, 0, 0)

# Scan if the laser scan, angle is in radians and dist is the distance to an object or threshold value.
def angle_is_safe(angle, dist, world_state, ros_util):

    # If everything in front of us is open, return true as this is safe to traverse.
    if np.isnan(np.nanmin(scan.ranges)):
        return True

    # Calculate how much to change angle in order for robot to clear obstacle
    buffer_angle = math.atan(buffer_safe / dist)

    # These are in radians.
    angle1 = angle - buffer_angle
    angle2 = angle + buffer_angle

    index1 = int((angle1 - scan.angle_min) / scan.angle_increment)
    index2 = int((angle2 - scan.angle_min) / scan.angle_increment)

    start = min(index1, index2)
    end = max(index1, index2)

    # This is the amount of nan values that we need to look for to see if we can fit through
    rospy.loginfo("Our epsilon array size is = {}".format(end - start))

    rospy.loginfo(scan.ranges)

    if start < 0 or end >= len(scan.ranges):
        rospy.loginfo("We are out of the scan.ranges array! Turning towards the area of uncertainty.")

        if start < 0:
            for i in range(0, end):
                # might need to fine-tune dist
                if (not np.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
                    rospy.loginfo("We cannot fit in the space seen in the given wedge! Return False.")
                    return False

            direction = 'right'
            directionBack = 'left'
            new_angle = start * scan.angle_increment + scan.angle_min + buffer_angle
        else:
            for i in range(start, len(scan.ranges) - 1):
                # might need to fine-tune dist
                if (not np.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
                    rospy.loginfo("We cannot fit in the space seen in the given wedge! Return False.")
                    return False

            direction = 'left'
            directionBack = 'right'
            new_angle = end * scan.angle_increment + scan.angle_min - buffer_angle

        old_heading = world_state.heading

        # Turn towards the area of uncertainty and check if it is open and then turn back.
        uf.turn(rel_to_abs(world_state.heading, new_angle), direction, world_state, ros_util)
        retVal = angle_is_safe(new_angle, dist, world_state, ros_util)
        uf.turn(old_heading, directionBack, world_state, ros_util)
        return retVal

    # rospy.loginfo("whole array: {}".format(scan.ranges))
    # test = []
    # for i in range(start, end):
    #     test.append(scan.ranges[i])
    # rospy.loginfo("epsilon array: {}".format(test))

    for i in range(start, end):
        # might need to fine-tune dist
        if (not np.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
            rospy.loginfo("We cannot fit in the space seen in the given wedge! Return False.")
            return False

    return True

def get_best_angle(world_state, ros_util):  #TODO: We need to account for the case where all nan's exist right at the goal!!!!
    best_total_dist = None
    best_angle = None
    best_x = None
    best_y = None

    for i in range(1, len(scan.ranges)):
        # Get range for current and previous LaserScan angles
        current = scan.ranges[i]
        previous = scan.ranges[i-1]

        # Both current and previous are NaN: no obstacle edge
        if math.isnan(current) and math.isnan(previous):
            continue
        # Only one of current and previous are NaN: obstacle edge
        elif math.isnan(current) or math.isnan(previous):
            if math.isnan(current):
                obst_to_safe = True
                dist = previous
                idx = i
            else:
                obst_to_safe = False
                dist = current
                idx = i-1
        # Neither are NaN: there must be sufficient difference for obstacle edge
        elif abs(current - previous) > dist_thresh:
            if current > previous:
                obst_to_safe = True
                dist = previous
                idx = i
            else:
                obst_to_safe = False
                dist = current
                idx = i-1
        # No obstacle edge in any other case
        else:
            continue

        # Calculate angle at this index
        d_angle = scan.angle_min + idx * scan.angle_increment

        # Calculate how much to change angle in order for robot to clear obstacle
        buffer_angle = math.atan(buffer / dist)

        # Add room for clearing obstacle
        if obst_to_safe:
            d_angle += buffer_angle
        else:
            d_angle -= buffer_angle

        # Scale distance so that robot gets to a point parallel to the obstacle
        dist = math.sqrt(buffer ** 2 + dist ** 2)

        if not angle_is_safe(d_angle, dist, world_state, ros_util):  # TODO: Should this be dist or a slightly greater value????
            continue

        # Add current heading to get angle in world reference
        total_angle = (world_state.heading * math.pi / 180) + d_angle

        # Calculate the change in X, Y to get to the edge of the obstacle
        d_x = dist * math.cos(total_angle)
        d_y = dist * math.sin(total_angle)

        # Calculate coordinates of the edge of the obstacle
        obst_x = world_state.positionX + d_x
        obst_y = world_state.positionY + d_y

        # Calculate heuristic: distance between the goal and the edge of the obstacle
        heur_dist = math.sqrt((world_state.target_location.x - obst_x) ** 2 + (world_state.target_location.y - obst_y) ** 2)

        # Save direction that minimizes veering off-track from our goal while avoiding obstacles
        if best_total_dist is None or (heur_dist + dist) < best_total_dist:
            best_total_dist = heur_dist + dist
            best_angle = d_angle
            best_x = obst_x
            best_y = obst_y

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