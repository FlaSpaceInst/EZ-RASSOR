import rospy
import math
import utility_functions as uf
import nav_functions as nf
import arm_force as armf
from sensor_msgs.msg import LaserScan

scan = None
threshold = 4.0

def on_scan_update(new_scan):
    global scan
    scan = new_scan

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
    while not at_target(world_state, ros_util):
        
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
        
        rospy.loginfo("angle2goal_radians: {}".format(angle2goal_radians))
        rospy.loginfo("scan.angle_max: {}".format(scan.angle_max))
        rospy.loginfo("scan.angle_min: {}".format(scan.angle_min))
        rospy.loginfo("world_state.heading: {}".format(world_state.heading))
        
        # if we can't see the goal, face it
        if angle2goal_radians > scan.angle_max or angle2goal_radians < scan.angle_min:
            rospy.loginfo("can't see the goal, turning")
            uf.turn(new_heading_degrees, direction, world_state, ros_util)
            continue
        
        scan_index = int((angle2goal_radians - scan.angle_min) / scan.angle_increment)
        
        if math.isnan(scan.ranges[scan_index]) or scan.ranges[scan_index] >= threshold:
            rospy.loginfo("passed threshold")
        else:
            # do wedgebug stuff
            # search for new relative angle
            rospy.loginfo("stopping")
            ros_util.publish_actions('stop', 0, 0, 0, 0)
            continue
            
            
        rospy.loginfo("we see goal, turning")
            
        uf.turn(new_heading_degrees, direction, world_state, ros_util)

        # Otherwise go forward
        ros_util.publish_actions('forward', 0, 0, 0, 0)
            
        ros_util.rate.sleep()
    
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
