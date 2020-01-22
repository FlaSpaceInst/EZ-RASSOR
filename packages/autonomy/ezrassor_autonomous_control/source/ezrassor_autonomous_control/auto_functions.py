import rospy
import math
import utility_functions as uf
import nav_functions as nf
import arm_force as armf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

scan = None
threshold = 4.0

# threshold that determines if a point in the laser scan is a discontinuity
discThresh = 3.0

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

        # Get new heading angle relative to current heading as (0,0)
        new_heading_degrees = nf.calculate_heading(world_state, ros_util)
        angle2goal_radians = nf.adjust_angle(world_state.heading, new_heading_degrees)
        
        if angle2goal_radians < 0:
            direction = 'right'
        else:
            direction = 'left'

        # If we are not facing the goal, turn to face it.
        if angle2goal_radians > scan.angle_max or angle2goal_radians < scan.angle_min:
            rospy.loginfo("can't see the goal, turning")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            continue
        
        scan_index = int((angle2goal_radians - scan.angle_min) / scan.angle_increment)
        
        if math.isnan(scan.ranges[scan_index]) or scan.ranges[scan_index] >= threshold:
            rospy.loginfo("passed threshold")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)

            # Otherwise go forward
            ros_util.publish_actions('forward', 0, 0, 0, 0)
        
        # If we enter here, there is an obstacle blocking our path. Use wedgebug to avoid obstacles.
        else:

            # If we can't see the goal directly, check for the best direction of travel
            bestAngle = 0.0
            besti = 0
            bestDist = 10000.0
            bestXDist = None
            bestYDist = None

            rospy.loginfo("scan.ranges: {}".format(scan.ranges))

            for i in range(len(scan.ranges)):
                # check for discontinuties within a specified threshold

                current = scan.ranges[i]
                previous = scan.ranges[i-1]

                # If the current or previous ray is nan, set its value to a very far away constant.
                if math.isnan(current):
                    current = 1000000
                if math.isnan(previous):
                    previous = 1000000

                if (i>0) and (abs(current - previous) > discThresh):

                    # output the index for the discontinuity and the angle value and the distance to that discontinuity
                    discDist = scan.ranges[i]

                    if discDist == float('nan'):
                        discDist = scan.range_max

                    dAng = scan.angle_min + i * scan.angle_increment
                    xDist = discDist * math.sin(dAng)
                    yDist = discDist * math.cos(dAng)
                    heurDist = math.sqrt((world_state.target_location.x - xDist) ** 2 + (world_state.target_location.y - yDist) ** 2)

                    if ((heurDist + discDist) < bestDist):
                        bestDist = heurDist + discDist
                        bestAngle = dAng
                        bestXDist = xDist
                        bestYDist = yDist

            rospy.loginfo("best i (ray): {}".format(scan.ranges[besti]))
            rospy.loginfo("best angle: {}".format(bestAngle))

            if bestAngle < 0:
                direction = 'right'
            else:
                direction = 'left'

            rospy.loginfo("Attempting to turn towards best heuristic!")
            rospy.loginfo("current heading: {}, new heading: {}".format(world_state.heading, rel_to_abs(world_state.heading, bestAngle)))
            uf.turn(rel_to_abs(world_state.heading, bestAngle), direction, world_state, ros_util)

            target_x = world_state.positionX + bestXDist
            target_y = world_state.positionY + bestYDist

            while not at_target(world_state.positionX, world_state.positionY,
                                target_x, target_y, ros_util.threshold):
                ros_util.publish_actions('forward', 0, 0, 0, 0)
            
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
