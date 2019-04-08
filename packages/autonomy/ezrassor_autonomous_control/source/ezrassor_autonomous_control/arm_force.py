import rospy
from ai_objects import WorldState, ROSUtility
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int8

''' Used for debugging '''
# import threading
EPSILON = 0.05


''' Checks if current arms force is near target '''
def is_force_target(world_state, target_force):
    front_force = target_force - EPSILON < world_state.state_flags['force_front_arm'] < target_force + EPSILON
    back_force = target_force - EPSILON < world_state.state_flags['force_back_arm'] < target_force + EPSILON
    return front_force and back_force

''' Given a target force sets both arms to the target '''
def set_target_force(world_state,ros_util,target_force):

    while not is_force_target(world_state,target_force):
        msg = 0b0
        if not target_force - EPSILON < world_state.state_flags['force_front_arm'] < target_force + EPSILON:
            if target_force > world_state.state_flags['force_front_arm']:
                msg += ros_util.commands['front_arm_up']
            elif target_force < world_state.state_flags['force_front_arm']:
                msg += ros_util.commands['front_arm_down']

        if not target_force - EPSILON < world_state.state_flags['force_back_arm'] < target_force + EPSILON:
            if target_force > world_state.state_flags['force_back_arm']:
                msg += ros_util.commands['back_arm_up']
            elif target_force < world_state.state_flags['force_back_arm']:
                msg += ros_util.commands['back_arm_down']
        msg += ros_util.commands['front_dig']
        msg += ros_util.commands['back_dig']
        ros_util.command_pub.publish(msg)
        ros_util.rate.sleep()

    ros_util.command_pub.publish(ros_util.commands['null'])




''' This is left here for debugging'''
# threading.Thread(target=lambda: rospy.init_node('force_pub', disable_signals=True)).start()
# world = WorldState()
# ros_util = ROSUtility()
# rospy.Subscriber('/ezrassor/joint_states', JointState, forceCallBack, callback_args=(world,))
# rospy.Subscriber('/ezrassor/routine_toggles', Int8, ros_util.autoCommandCallBack)
