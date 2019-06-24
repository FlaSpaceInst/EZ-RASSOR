import rospy
import sys

from std_msgs.msg import Int8, Int16, String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu

from numpy import random as r

import ai_objects as obj
import auto_functions as af
import utility_functions as uf

def on_start_up(target_x, target_y, movement_topic, front_arm_topic, 
                back_arm_topic, front_drum_topic, back_drum_topic,
                max_linear_velocity=1, max_angular_velocity=1, 
                real_odometry=False):
    """ Initialization Function  """

    # ROS Node Init Parameters 
    rospy.init_node('autonomous_control', anonymous=True)
    
    #Create Utility Objects
    world_state = obj.WorldState()
    ros_util = obj.ROSUtility(movement_topic, 
                              front_arm_topic,
                              back_arm_topic, 
                              front_drum_topic,
                              back_drum_topic, 
                              max_linear_velocity, 
                              max_angular_velocity)

    target_location = Point()
    temp = Point()

    target_location.x = target_x
    target_location.y = target_y

    temp.x = target_x
    temp.y = target_y

    world_state.target_location = target_location
    world_state.dig_site = temp

    ros_util.status_pub.publish('Spinning Up AI Control')

    # Setup Subscriber Callbacks
    if real_odometry:
        # This topic will be changed to represent whatever
        # topic the odometry data is being published to
        print("Real Odometry")
        rospy.Subscriber('stereo_odometer/odometry', 
                         Odometry, 
                         world_state.odometryCallBack)
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
    rospy.Subscriber('obstacle_detect', 
                     Int8, 
                     world_state.visionCallBack)
    rospy.Subscriber('autonomous_toggles', 
                     Int8, 
                     ros_util.autoCommandCallBack)

    uf.set_back_arm_angle(world_state, ros_util, 1.5)
    uf.set_front_arm_angle(world_state, ros_util, 1.5)
    
    autonomous_control_loop(world_state, ros_util)


def full_autonomy(world_state, ros_util):
    """ Full Autonomy Loop Function """ 
    
    ros_util.status_pub.publish('Full Autonomy Activated.')

    while(True):
        print(world_state.dig_site)
        af.auto_drive_location(world_state, ros_util)
        af.auto_dig(world_state, ros_util, 7)
        af.auto_dock(world_state, ros_util)
        af.auto_dump(world_state, ros_util, 4)
        world_state.target_location.x = world_state.dig_site.x
        world_state.target_location.y = world_state.dig_site.y
    

def autonomous_control_loop(world_state, ros_util):
    """ Control Auto Functions based on auto_function_command input. """
    
    print("Entered Control Loop")
    print(ros_util.auto_function_command)

    while(True):

        while ros_util.auto_function_command == 0:
            ros_util.publish_actions('stop', 0, 0, 0, 0)
            ros_util.rate.sleep()

        ros_util.control_pub.publish(True)

        if ros_util.auto_function_command == 1:
            af.auto_drive_location(world_state, ros_util)
        elif ros_util.auto_function_command == 2:
            af.auto_dig(world_state, ros_util, 10)
        elif ros_util.auto_function_command == 4:
            af.auto_dump(world_state, ros_util, 10)
        elif ros_util.auto_function_command == 8:
            uf.self_right_from_side(world_state, ros_util)
        elif ros_util.auto_function_command == 16:
            full_autonomy(world_state, ros_util)
        else:
            ros_util.status_pub.publish('Error Incorrect Auto Function Request {}'
                                        .format(ros_util.auto_function_command))
        
        ros_util.publish_actions('stop', 0, 0, 0, 0)
        ros_util.control_pub.publish(False)
