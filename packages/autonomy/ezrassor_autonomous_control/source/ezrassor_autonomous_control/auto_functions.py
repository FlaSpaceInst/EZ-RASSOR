import rospy
import math
import utility_functions as uf
import nav_functions as nf
import arm_force as armf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import copy

scan = None
threshold = 4.0

# threshold that determines if a point in the laser scan is a discontinuity
dist_thresh = 3.0
buffer = 1.5

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
        
        if uf.self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        # Get new heading angle relative to current heading
        new_heading_degrees = nf.calculate_heading(world_state, ros_util)
        angle2goal_radians = nf.adjust_angle(world_state.heading, new_heading_degrees)
        
        if angle2goal_radians < 0:
            direction = 'right'
        else:
            direction = 'left'

        # Create a copy of the current scan so the values don't change later on
        scan_copy = copy.deepcopy(scan)

        # If we are not facing the goal, turn to face it.
        if angle2goal_radians > scan_copy.angle_max or angle2goal_radians < scan_copy.angle_min:
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            continue
        
        scan_index = int((angle2goal_radians - scan_copy.angle_min) / scan_copy.angle_increment)
        
        # If we can see the goal (no obstacles in the way), go towards it
        if math.isnan(scan_copy.ranges[scan_index]) or scan_copy.ranges[scan_index] >= threshold:
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            ros_util.publish_actions('forward', 0, 0, 0, 0)
        # If we enter here, there is an obstacle blocking our path. Use wedgebug to avoid obstacles.
        else:
            best_total_dist = None

            # Find best direction to travel to get to goal while avoiding obstacles.
            # This is done by finding the edge of the obstacle that would allow the robot to
            # minimize the distance it travels away from the goal while avoiding the obstacle.
            for i in range(1, len(scan_copy.ranges)):
                # Get range for current and previous LaserScan angles
                current = scan_copy.ranges[i]
                previous = scan_copy.ranges[i-1]

                # Both current and previous are NaN: no obstacle edge
                if math.isnan(current) and math.isnan(previous):
                    continue
                # Only one of current and previous are NaN: obstacle edge
                elif math.isnan(current) or math.isnan(previous):
                    if math.isnan(current):
                        obst_to_safe = True
                        dist = previous
                        idx = i-1
                    else:
                        obst_to_safe = False
                        dist = current
                        idx = i
                # Neither are NaN: there must be sufficient difference for obstacle edge
                elif abs(current - previous) > dist_thresh:
                    if current > previous:
                        obst_to_safe = True
                        dist = previous
                        idx = i-1
                    else:
                        obst_to_safe = False
                        dist = current
                        idx = i
                # No obstacle edge in any other case
                else:
                    continue

                # Calculate angle at this index
                d_angle = scan_copy.angle_min + idx*scan_copy.angle_increment

                # Calculate how much to change angle in order for robot to clear obstacle
                buffer_angle = math.atan(buffer/dist)

                # Add room for clearing obstacle
                if obst_to_safe:
                    d_angle += buffer_angle
                else:
                    d_angle -= buffer_angle

                # Add current heading to get angle in world reference
                total_angle = (world_state.heading*math.pi/180.) + d_angle

                # Calculate the change in X, Y to get to the edge of the obstacle
                d_x = dist * math.cos(total_angle)
                d_y = dist * math.sin(total_angle)

                # Calculate coordinates of the edge of the obstacle
                obst_x = world_state.positionX + d_x
                obst_y = world_state.positionY + d_y

                # Calculate heuristic: distance between the goal and the edge of the obstacle
                heur_dist = math.sqrt((world_state.target_location.x - obst_x)**2 + (world_state.target_location.y - obst_y)**2)

                # Save direction that minimizes veering off-track from our goal while avoiding obstacles
                if best_total_dist is None or (heur_dist + dist) < best_total_dist:
                    best_total_dist = heur_dist + dist
                    best_i = idx
                    best_angle = d_angle
                    best_x = obst_x
                    best_y = obst_y

            rospy.loginfo("ranges: {}".format(scan_copy.ranges))
            rospy.loginfo("best direction: {}".format(best_angle))
            rospy.loginfo("range for best angle: {}".format(scan_copy.ranges[best_i]))
            rospy.loginfo("current coordinates: ({}, {})".format(world_state.positionX, world_state.positionY))
            rospy.loginfo("target coordinates: ({}, {})".format(best_x, best_y))
            rospy.loginfo("current heading: {}, new heading: {}".format(world_state.heading, rel_to_abs(world_state.heading, best_angle)))

            if best_angle < 0:
                direction = 'right'
            else:
                direction = 'left'

            # Turn towards the direction chosen
            uf.turn(rel_to_abs(world_state.heading, best_angle), direction, world_state, ros_util)

            rospy.loginfo("heading after turning: {}".format(world_state.heading))

            while not at_target(world_state.positionX, world_state.positionY,
                                best_x, best_y, ros_util.threshold):
                ros_util.publish_actions('forward', 0, 0, 0, 0)

            rospy.loginfo('Reached edge of obstacle')
            
        ros_util.rate.sleep()

    rospy.loginfo('Destination reached!')
    ros_util.publish_actions('stop', 0, 0, 0, 0)
        
def rel_to_abs(current_heading_degrees, relative_heading_radians):
    relative_heading_degrees = (180 * relative_heading_radians / math.pi)
    abs_heading = relative_heading_degrees + current_heading_degrees

    if abs_heading < 0:
        new_heading = 360 + abs_heading

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
