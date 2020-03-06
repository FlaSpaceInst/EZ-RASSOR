import rospy
from std_msgs.msg import String, Float32, Bool
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Twist
import nav_functions as nf
import math
import rospkg
from os import listdir
from os.path import isfile, join
import re

from random import uniform

class WorldState():
    """ World State Object Representing
        All Sensor Data
    """

    def __init__(self):
        self.positionX = 0
        self.positionY = 0
        self.positionZ = 0
        self.startPositionX = 0
        self.startPositionY = 0
        self.originZ = 0
        self.front_arm_angle = 0
        self.back_arm_angle = 0
        self.front_arm_angle = 0
        self.heading = 0
        self.warning_flag = 0
        self.target_location = Point()
        self.on_side = False
        self.on_back = False
        self.front_up = False
        self.back_up = False
        self.battery = 100
        self.hardware_status = True


    def jointCallBack(self, data):
        """ Set state_flags joint position data. """

        self.front_arm_angle = data.position[1]
        self.back_arm_angle = data.position[0]

    def odometryCallBack(self, data):
        """ Set state_flags world position data. """
        
        self.positionX = data.pose.pose.position.x + self.startPositionX
        self.positionY = data.pose.pose.position.y + self.startPositionY

        heading = nf.quaternion_to_yaw(data.pose.pose) * 180/math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading

    def simStateZPositionCallBack(self, data):
        """ More accurate position data to use for
            testing and experimentation.
        """
        index = 0

        namespace = rospy.get_namespace()
        namespace = namespace[1:-1]+"::base_link"
        try:
            index = data.name.index(namespace)
        except Exception:
            rospy.logdebug("Failed to get index. Skipping...")
            return

        self.positionZ = data.pose[index].position.z + self.originZ

    def simStateCallBack(self, data):
        """ More accurate position data to use for
            testing and experimentation.
        """
        index = 0

        namespace = rospy.get_namespace()
        namespace = namespace[1:-1]+"::base_link"
        try:
            index = data.name.index(namespace)
        except Exception:
            rospy.logdebug("Failed to get index. Skipping...")
            return

        self.positionX = data.pose[index].position.x
        self.positionY = data.pose[index].position.y

        heading = nf.quaternion_to_yaw(data.pose[index]) * 180/math.pi

        if heading > 0:
            self.heading = heading
        else:
            self.heading = 360 + heading

    def imuCallBack(self, data):
        " Heading data collected from orientation IMU data. "

        if abs(data.linear_acceleration.y) > 9:
            self.on_side = True
        else:
            self.on_side = False

    def visionCallBack(self, data):
        """ Set state_flags vision data. """

        self.warning_flag = data.data

    def get_arm_force(self):
        front_arm_force = self.state_flags['front_arm_angle'] + .2 + uniform(-.2, .2)
        back_arm_force = self.state_flags['back_arm_angle'] + .2 + uniform(-.2, .2)
        return front_arm_force, back_arm_force

    # Attempts to get initial elevation from file in dem_data/
    def get_origin_dem_data(self, directory):

        # Use list comprehension to get only files in directory as opposed to files and subdirectories
        onlyfiles = [f for f in listdir(directory) if isfile(join(directory, f))]

        # User hasn't put file in dem_data
        if not onlyfiles:
            rospy.logerr("No elevation file, initial z defaulting to 0")
        else:
            rospy.loginfo("Reading %s", onlyfiles[0])
            file = open(directory + onlyfiles[0], "r")
            middle = -1

            # Reads file line by line, line number starts at 0
            for i, line in enumerate(file):

                # 3rd line of file contains "(rows, cols)"
                if i == 2:

                    # Use regex to obtain dimmensions
                    dem_size = map(int,re.findall(r'-?(\d+)',line))
                    rospy.loginfo(dem_size)

                    # File doesn't have dimmensions at line 3
                    if not dem_size:
                        rospy.logerr("Couldn't find dem size")
                        break
                    else:

                        # Give warning if not square
                        if dem_size[0] != dem_size[1]:
                            rospy.logwarn("Dimmensions are not same value (w != l). Treating as w x w")

                        # Get the indices for the origin (gazebo's origin is in the middle)
                        middle = int(dem_size[0] / 2)
                        rospy.loginfo("Dem size: {}, middle: {}".format(dem_size[0], middle))

                # If we have a middle index and on the expected line, the "+ 3" is to offset
                # The first three lines are: title, corner (lat, long) coordinates, and size
                if middle != -1 and i == middle + 3:

                    #  Split by white space, then find the middle value on the level
                    temp = line.split()
                    rospy.loginfo("Dem center value: %s", temp[middle])
                    self.originZ = float(temp[middle])

                    # We found what we're looking for so we stop
                    break

    # Find the path to dem_data/
    def path_dem(self):
        rospack = rospkg.RosPack()
        base = rospack.get_path("ezrassor_autonomous_control")
        return base + "/dem_data/"

    # Use initial spawn coordinates to later offset position
    def initial_spawn(self, start_x, start_y):
        self.startPositionX = start_x
        self.startPositionY = start_y

class ROSUtility():
    """ ROS Utility class that provides publishers,
        subscribers, and convinient ROS utilies.
    """

    def __init__(self, movement_topic, front_arm_topic, back_arm_topic,
                 front_drum_topic, back_drum_topic,
                 max_linear_velocity, max_angular_velocity):
        """ Initialize the ROS Utility Object. """

        self.movement_pub = rospy.Publisher(movement_topic,
                                            Twist,
                                            queue_size=10)
        self.front_arm_pub = rospy.Publisher(front_arm_topic,
                                             Float32,
                                             queue_size=10)
        self.back_arm_pub = rospy.Publisher(back_arm_topic,
                                            Float32,
                                            queue_size=10)
        self.front_drum_pub = rospy.Publisher(front_drum_topic,
                                              Float32,
                                              queue_size=10)
        self.back_drum_pub = rospy.Publisher(back_drum_topic,
                                             Float32,
                                             queue_size=10)
        self.control_pub = rospy.Publisher('secondary_override_toggle',
                                           Bool,
                                           queue_size=10)
        self.rate = rospy.Rate(45) # 10hz

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self.auto_function_command = 0

        self.threshold = .5

    def publish_actions(self, movement, front_arm, back_arm,
                        front_drum, back_drum):
        """ Publishes actions for all joints and motors """

        twist_message = Twist()

        if movement == 'forward':
            twist_message.linear.x = self.max_linear_velocity
        elif movement == 'reverse':
            twist_message.linear.x = -self.max_linear_velocity
        elif movement == 'left':
            twist_message.angular.z = self.max_angular_velocity
        elif movement == 'right':
            twist_message.angular.z = -self.max_angular_velocity
        else:
            pass

        self.movement_pub.publish(twist_message)
        self.front_arm_pub.publish(front_arm)
        self.back_arm_pub.publish(back_arm)
        self.front_drum_pub.publish(front_drum)
        self.back_drum_pub.publish(back_drum)

    def autoCommandCallBack(self, data):
        """ Set auto_function_command to the current choice. """
        self.auto_function_command = data.data
