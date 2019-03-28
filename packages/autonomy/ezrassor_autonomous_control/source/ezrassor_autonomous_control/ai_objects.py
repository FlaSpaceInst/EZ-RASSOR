import rospy
from std_msgs.msg import Int8, Int16, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from nav_functions import quaternion_to_yaw


class WorldState():
    """ World State Object Representing All Sensor Data """

    def __init__(self):
        self.kill_bit = 0b1000000000000
        self.state_flags = {'positionX': 0, 'positionY': 0, 'positionZ': 0, 
                            'front_arm_angle': 0, 'back_arm_angle': 0, 
                            'front_arm_angle': 0, 'heading': 0, 'warning_flag': 0,
                            'target_location': [10,10], 'on_side': False}

        self. auto_function_command = 0


    def jointCallBack(self, data):
        """ Set state_flags joint position data. """

        self.state_flags['front_arm_angle'] = -(data.position[1])
        self.state_flags['back_arm_angle'] = data.position[0]
    

    def odometryCallBack(self, data):
        """ Set state_flags world position data. """

        self.state_flags['positionX'] = data.pose.pose.position.z
        self.state_flags['positionY'] = data.pose.pose.position.y
        self.state_flags['heading'] = quaternion_to_yaw(data.pose.pose.orientation)

    def simStateCallBack(self, data):
        """ More accurate position data to use for testing and experimentation. """

        self.state_flags['positionX'] = data.pose[1].position.x
        self.state_flags['positionY'] = data.pose[1].position.y
        
        self.state_flags['heading'] = quaternion_to_yaw(data.pose[1])

    def imuCallBack(self, data):
        " Heading data collected from orientation IMU data. "

        if abs(data.linear_acceleration.y) > 9:
            self.state_flags['on_side'] = True
        else:
            self.state_flags['on_side'] = False

        #self.state_flags['heading'] = quaternion_to_euler(data.pose.orientation)

    def visionCallBack(self, data):
        """ Set state_flags vision data. """

        self.state_flags['warning_flag'] = data.data


class ROSUtility():

    def __init__(self):
        self.command_pub = rospy.Publisher('ezrassor/routine_responses', Int16, queue_size=100)
        self.status_pub = rospy.Publisher('ez_rassor/status', String, queue_size=100)
        self.rate = rospy.Rate(30) # 30hz

        self. auto_function_command = 0

        self.commands = {'forward' : 0b101000000000, 'reverse' : 0b010100000000, 'left' : 0b011000000000, 'right' : 0b100100000000, 
                'front_arm_up' : 0b000010000000, 'front_arm_down' : 0b000001000000, 'back_arm_up' : 0b000000100000, 'back_arm_down' : 0b000000010000,
                'front_dig' : 0b000000001000, 'front_dump' : 0b000000000100, 'back_dig' : 0b000000000010, 'back_dump' : 0b000000000001,
                'arms_up' : 0b000010100000, 'arms_down' : 0b000001010000, 'null': 0b000000000000}

    def autoCommandCallBack(self, data):
        """ Set auto_function_command to the current choice. """
        self.auto_function_command = data.data



