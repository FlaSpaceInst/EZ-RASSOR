import rospy
import ai_objects as obj
import auto_functions as af
import utility_functions as uf
from std_msgs.msg import Int8, Int16, String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState, Imu, LaserScan

""" Initialization Function  """
def on_start_up(target_x, target_y, start_x, start_y, movement_topic,
                front_arm_topic, back_arm_topic, front_drum_topic,
                back_drum_topic, obstacle_threshold, obstacle_buffer,
                move_increment, max_linear_velocity=1, max_angular_velocity=1,
                real_odometry=False):

    # ROS Node Init Parameters
    rospy.init_node('autonomous_control')

    # Create Utility Objects
    world_state = obj.WorldState()
    ros_util = obj.ROSUtility(movement_topic,
                              front_arm_topic,
                              back_arm_topic,
                              front_drum_topic,
                              back_drum_topic,
                              max_linear_velocity,
                              max_angular_velocity,
                              obstacle_threshold,
                              obstacle_buffer,
                              move_increment)

    target_location = Point()
    temp = Point()

    target_location.x = target_x
    target_location.y = target_y

    temp.x = target_x
    temp.y = target_y

    world_state.target_location = target_location
    world_state.dig_site = temp

    # Setup Subscriber Callbacks

    # If enable_real_odometry flag is true, use odometry for position
    if real_odometry:
        # Get initial spawn coords
        world_state.initial_spawn(start_x, start_y)

        # Get x and y from filtered odometry readings
        rospy.Subscriber('odometry/filtered',
                         Odometry,
                         world_state.odometryCallBack)

    # If enable_real_odometry flag is false, just use Gazebo's position
    else:
        rospy.Subscriber('/gazebo/link_states',
                         LinkStates,
                         world_state.simStateCallBack)

    rospy.Subscriber('imu',
                     Imu,
                     world_state.imuCallBack)
    rospy.Subscriber('joint_states',
                     JointState,
                     world_state.jointCallBack)
    rospy.Subscriber('autonomous_toggles',
                     Int8,
                     ros_util.autoCommandCallBack)
    rospy.Subscriber('obstacle_detection/combined',
                     LaserScan,
                     uf.on_scan_update,
                     queue_size=1)

    rospy.loginfo('Autonomous control initialized.')

    autonomous_control_loop(world_state, ros_util)

def full_autonomy(world_state, ros_util):
    """ Full Autonomy Loop Function """

    rospy.loginfo('Full autonomy activated.')

    while(ros_util.auto_function_command == 16):
        af.auto_drive_location(world_state, ros_util)
        if ros_util.auto_function_command != 16:
            break
        af.auto_dig(world_state, ros_util, 7)
        if ros_util.auto_function_command != 16:
            break
        af.auto_dock(world_state, ros_util)
        if ros_util.auto_function_command != 16:
            break
        af.auto_dump(world_state, ros_util, 4)
        world_state.target_location.x = world_state.dig_site.x
        world_state.target_location.y = world_state.dig_site.y

    world_state.target_location.x = world_state.dig_site.x
    world_state.target_location.y = world_state.dig_site.y

def autonomous_control_loop(world_state, ros_util):
    """ Control Auto Functions based on auto_function_command input. """

    while(True):
        while ros_util.auto_function_command == 0 or ros_util.auto_function_command == 32:
            ros_util.publish_actions('stop', 0, 0, 0, 0)
            ros_util.rate.sleep()

        ros_util.control_pub.publish(True)

        if ros_util.auto_function_command == 1:
            af.auto_drive_location(world_state, ros_util)
        elif ros_util.auto_function_command == 2:
            af.auto_dig(world_state, ros_util, 10)
        elif ros_util.auto_function_command == 4:
            af.auto_dump(world_state, ros_util, 4)
        elif ros_util.auto_function_command == 8:
            af.auto_dock(world_state, ros_util)
        elif ros_util.auto_function_command == 16:
            full_autonomy(world_state, ros_util)
        else:
            rospy.loginfo('Invalid auto-function request: %s.',
                          str(ros_util.auto_function_command))

        ros_util.auto_function_command = 0
        ros_util.publish_actions('stop', 0, 0, 0, 0)
        ros_util.control_pub.publish(False)
