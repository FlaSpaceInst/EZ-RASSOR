import rospy
import math
from tf import transformations


def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """
    
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def calculate_heading(world_state, ros_util):
    """ Calculate the new heading of the robot given its current location and the target location. """

    y2, y1, x2, x1 = world_state.state_flags['target_location'][0], world_state.state_flags['positionY'], world_state.state_flags['target_location'][1], world_state.state_flags['positionX']

    new_heading = math.atan2(y2-y1, x2-x1)
    new_heading = int(360*new_heading/math.pi)
    ros_util.status_pub.publish("New Heading: {}".format(new_heading))

    return new_heading
    
def adjust_angle(heading, new_heading):
    """ Adjust the angle difference to determine fastest turning direction to reach the goal. """
    
    angle_difference = new_heading - heading
    angle_difference = (angle_difference + 180) % 360 - 180
    
    return angle_difference

def quaternion_to_yaw(pose):
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    euler = transformations.euler_from_quaternion(quaternion)
    
    return euler[2]

