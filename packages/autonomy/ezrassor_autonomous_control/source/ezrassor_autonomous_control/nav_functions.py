import rospy
import math
from tf import transformations

def euclidean_distance(x1, x2, y1, y2):
    """ Calculate Euclidean distance from (x1,y1) to (x2,y2). """
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def calculate_heading(world_state):
    # Calculate the new heading of the robot given its current location and the
    # target location.
    y2 = world_state.target_location.y
    y1 = world_state.positionY
    x2 = world_state.target_location.x
    x1 = world_state.positionX

    # This is the x and y distances.
    dy = y2 - y1
    dx = x2 - x1

    # Get the angle in radians.
    new_heading = math.atan2(dy, dx)

    # Convert the angle to degrees.
    new_heading = (180 * new_heading/math.pi)

    # Since gazebo only uses 0 through 360 degree, we must bound any negative
    # angles to positive ones.
    if new_heading < 0:
        new_heading = 360 + new_heading

    return new_heading

def adjust_angle(heading, new_heading):
    # Adjust the angle difference to determine fastest turning direction to
    # reach the goal.
    angle_difference = new_heading - heading
    angle_difference = (angle_difference + 180) % 360 - 180
    return (math.pi * angle_difference / 180)

def quaternion_to_yaw(pose):
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                  pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    return euler[2]

""" Returns whether a given angle is safe for the robot to go towards.

Check if EZ-RASSOR can fit through a space given the angle to turn towards it.
Angle is in radians and dist is the distance to an object or threshold value
passed in.
"""
def angle_is_safe(angle, dist, buffer, scan, threshold):

    '''# Calculate how much to change angle in order for robot to clear obstacle.
    buffer_angle = math.atan(buffer / threshold)

    # These are in radians.
    angle1 = angle - buffer_angle
    angle2 = angle + buffer_angle

    # These are the indices in our scan.ranges array that correspond to the
    # angles above.
    index1 = int((angle1 - scan.angle_min) / scan.angle_increment)
    index2 = int((angle2 - scan.angle_min) / scan.angle_increment)

    # Indices used to check if EZ-RASSOR can fit through the given area.
    start = min(index1, index2)
    end = max(index1, index2)

    # If we are looking outside of our current wedge, attempt to turn towards
    # the unseen area and return if it is safe.
    if start < 0 or end >= len(scan.ranges):
        return False

    # Check to see if something is blocking where we are trying to fit through.
    for i in range(start, end):
        if (not math.isnan(scan.ranges[i])) and scan.ranges[i] <= dist:
            return False'''

    return True

""" Returns the best angle to go towards in the given LaserScan

Iterate through the laser scan in our current view (wedge) and determine our
best move towards the goal. Aim to minimize the difference between our facing
angle and the angle to the goal while avoiding unsafe moves.
"""
def get_best_angle(world_state, buffer, scan, threshold):
    best_score = None
    best_angle = None

    for i in range(0, len(scan.ranges)):
        # Convert LaserScan index to angle (in radians)
        angle = scan.angle_min + i * scan.angle_increment

        # Make sure this angle is safe before considering it
        if not angle_is_safe(angle, threshold, buffer, scan, threshold):
            continue

        # Aim to minimize the difference between our facing angle and the angle
        # to the goal
        new_heading_degrees = calculate_heading(world_state)
        angle2goal_radians = adjust_angle(world_state.heading,
                                          new_heading_degrees)
        score = abs(angle2goal_radians - angle)
        if best_score is None or score < best_score:
            best_score = score
            best_angle = angle

    return best_angle

""" Converts the given relative angle to an absolute angle

Given an angle in degrees relative to the robot's current facing angle, this
function returns its absolute heading in degrees (same heading that the Gazebo
world uses).
"""
def rel_to_abs(current_heading_degrees, relative_heading_radians):
    # Convert from radians to degrees
    relative_heading_degrees = (180 * relative_heading_radians / math.pi)

    # Add current heading to relative heading to get absolute heading
    abs_heading = relative_heading_degrees + current_heading_degrees

    if abs_heading < 0:
        abs_heading += 360

    return abs_heading
