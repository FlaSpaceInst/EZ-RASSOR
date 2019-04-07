import rospy
from std_msgs.msg import String, Float32, Bool
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Twist
import nav_functions as nf
import math

class WorldState():
    """ World State Object Representing 
        All Sensor Data 
    """

    def __init__(self):
        self.positionX = 0
        self.positionY = 0
        self.positionZ = 0
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

        self. auto_function_command = 0


    def jointCallBack(self, data):
        """ Set state_flags joint position data. """

        self.front_arm_angle = -(data.position[1])
        self.back_arm_angle = data.position[0]
    

    def odometryCallBack(self, data):
        """ Set state_flags world position data. """

        self.positionX = data.pose.pose.position.z
        self.positionY = data.pose.pose.position.y
        self.heading = nf.quaternion_to_yaw(data.pose.pose.orientation)

    def simStateCallBack(self, data):
        """ More accurate position data to use for 
            testing and experimentation. 
        """

        self.positionX = data.pose[1].position.x
        self.positionY = data.pose[1].position.y
        
        self.state_flags['positionX'] = data.pose[2].position.x
        self.state_flags['positionY'] = data.pose[2].position.y
        
        heading = nf.quaternion_to_yaw(data.pose[2]) * 180/math.pi

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


class ROSUtility():
    """ ROS Utility class that provides publishers,
        subscribers, and convinient ROS utilies.
    """

    def __init__(self, movement_topic, arms_topic, drums_topic, 
                 max_linear_velocity, max_angular_velocity):
        """  """
        
        self.movement_pub = rospy.Publisher(movement_topic, 
                                            Twist, 
                                            queue_size=10)
        self.front_arm_pub = rospy.Publisher('front_'+arms_topic, 
                                             Float32, 
                                             queue_size=10)
        self.back_arm_pub = rospy.Publisher('back_'+arms_topic, 
                                            Float32, 
                                            queue_size=10)
        self.front_drum_pub = rospy.Publisher('front_'+drums_topic, 
                                              Float32, 
                                              queue_size=10)
        self.back_drum_pub = rospy.Publisher('back_'+drums_topic, 
                                             Float32, 
                                             queue_size=10)

        self.status_pub = rospy.Publisher('status', 
                                          String, 
                                          queue_size=10)
        self.control_pub = rospy.Publisher('autonomous_override_toggle', 
                                           Bool, 
                                           queue_size=10)
        self.rate = rospy.Rate(45) # 10hz

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self. auto_function_command = 0

    def publish_actions(self, movement, front_arm, back_arm, 
                        front_drum, back_drum):
        """ Publishes actions for all joints and motors """

        twist_message = Twist()

        if movement == 'forward':
            twist_message.linear.x = max_linear_velocity
        elif movement == 'reverse':
            twist_message.linear.x = -max_linear_velocity
        elif movement == 'left':
            twist_message.angular.z = max_angular_velocity
        elif movement == 'right':
            twist_message.angular.z = -max_angular_velocity
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



