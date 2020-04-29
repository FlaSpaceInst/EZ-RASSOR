import rospy
import nav_functions as nf
import math
from random import uniform
from std_msgs.msg import String, Float32, Bool
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Twist

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

    def get_arm_force(self):
        front_arm_force = self.state_flags['front_arm_angle'] + .2 + uniform(-.2, .2)
        back_arm_force = self.state_flags['back_arm_angle'] + .2 + uniform(-.2, .2)
        return front_arm_force, back_arm_force

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
                 max_linear_velocity, max_angular_velocity, obstacle_threshold,
                 obstacle_buffer, move_increment):
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
        self.arms_up_pub = rospy.Publisher('arms_up',
                                           Bool,
                                           queue_size=10)
        self.rate = rospy.Rate(45) # 10hz

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self.auto_function_command = 0

        self.threshold = .5

        self.obstacle_threshold = obstacle_threshold
        self.obstacle_buffer = obstacle_buffer
        self.move_increment = move_increment

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
