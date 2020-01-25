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
discThresh = 3.0
buffer = 1

def on_scan_update(new_scan):
    ranges = [float("nan")] * len(new_scan.ranges)
    for i in range(1, len(new_scan.ranges)-1):
        if math.isnan(new_scan.ranges[i]) and not math.isnan(new_scan.ranges[i-1]) and not math.isnan(new_scan.ranges[i+1]):
            # rospy.loginfo("replacing NaN")
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
        
        # rospy.loginfo("current heading: {}".format(world_state.heading))
        if uf.self_check(world_state, ros_util) != 1:
            rospy.logdebug('Status check failed.')
            return

        # Get new heading angle relative to current heading as (0,0)
        new_heading_degrees = nf.calculate_heading(world_state, ros_util)
        angle2goal_radians = nf.adjust_angle(world_state.heading, new_heading_degrees)
        
        if angle2goal_radians < 0:
            direction = 'right'
        else:
            direction = 'left'

        scan_copy = copy.deepcopy(scan)

        # If we are not facing the goal, turn to face it.
        if angle2goal_radians > scan_copy.angle_max or angle2goal_radians < scan_copy.angle_min:
            rospy.loginfo("can't see the goal, turning")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            continue
        
        scan_index = int((angle2goal_radians - scan_copy.angle_min) / scan_copy.angle_increment)
        
        if math.isnan(scan_copy.ranges[scan_index]) or scan_copy.ranges[scan_index] >= threshold:
            rospy.loginfo("passed threshold")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)

            # Otherwise go forward
            ros_util.publish_actions('forward', 0, 0, 0, 0)
        
        # If we enter here, there is an obstacle blocking our path. Use wedgebug to avoid obstacles.
        else:

            # If we can't see the goal directly, check for the best direction of travel
            bestDist = 10000.0

            rospy.loginfo("scan_copy.ranges: {}".format(scan_copy.ranges))

            for i in range(1, len(scan_copy.ranges)):
                # check for discontinuties within a specified threshold

                current = scan_copy.ranges[i]
                previous = scan_copy.ranges[i-1]

                # both current and previous are NaN: no discontinuity
                if math.isnan(current) and math.isnan(previous):
                    continue
                # only one of current and previous are NaN: discontinuity
                elif math.isnan(current) or math.isnan(previous):
                    if math.isnan(current):
                        obst_to_safe = True
                        discDist = previous
                        idx = i-1
                    else:
                        obst_to_safe = False
                        discDist = current
                        idx = i
                # neither are NaN: must be sufficient difference for discontinuity
                elif abs(current - previous) > discThresh:
                    if current > previous:
                        obst_to_safe = True
                        discDist = previous
                        idx = i-1
                    else:
                        obst_to_safe = False
                        discDist = current
                        idx = i
                # no discontinuity in any other case
                else:
                    continue

                dAng = scan_copy.angle_min + idx*scan_copy.angle_increment
#                if obst_to_safe:
#                    rospy.loginfo("adding buffer for {}".format(discDist))
#                    dAng += math.atan(buffer/discDist)
#                else:
#                    rospy.loginfo("subtracting buffer for {}".format(discDist))
#                    dAng -= math.atan(buffer/discDist)
                total_angle = (world_state.heading*math.pi/180.) + dAng
                dX = discDist * math.sin(total_angle)
                dY = discDist * math.cos(total_angle)
                obst_x = world_state.positionX + dX
                obst_y = world_state.positionY + dY

                heurDist = math.sqrt((world_state.target_location.x - obst_x)**2 + (world_state.target_location.y - obst_y)**2)

                if (heurDist + discDist) < bestDist:
                    bestDist = heurDist + discDist
                    besti = idx
                    bestAngle = dAng
                    bestX = obst_x
                    bestY = obst_y

            rospy.loginfo("best i (ray): {}".format(scan_copy.ranges[besti]))
            rospy.loginfo("best angle: {}".format(bestAngle))
            rospy.loginfo("best angle (abs): {}".format(rel_to_abs(world_state.heading, bestAngle)))
            rospy.loginfo("current x: {}, current y: {}".format(world_state.positionX, world_state.positionY))
            rospy.loginfo("target x: {}, target y: {}".format(bestX, bestY))

            if bestAngle < 0:
                direction = 'right'
            else:
                direction = 'left'

            rospy.loginfo("Attempting to turn towards best heuristic!")
            rospy.loginfo("current heading: {}, new heading: {}".format(world_state.heading, rel_to_abs(world_state.heading, bestAngle)))
            uf.turn(rel_to_abs(world_state.heading, bestAngle), direction, world_state, ros_util)
            rospy.loginfo("heading after turning: {}".format(world_state.heading))

            while not at_target(world_state.positionX, world_state.positionY,
                                obst_x, obst_y, ros_util.threshold):
#                if world_state.positionX > 5 or world_state.positionY > 5:
#                    rospy.loginfo("current position: ({}, {})".format(world_state.positionX, world_state.positionY))
                ros_util.publish_actions('forward', 0, 0, 0, 0)
            rospy.loginfo("reached goal!!")
            
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
