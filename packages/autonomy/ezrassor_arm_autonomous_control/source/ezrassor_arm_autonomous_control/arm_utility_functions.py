import rospy
from gazebo_ros_link_attacher.srv import Attach


def AttachPaver(flag, paver_count):
    rospy.wait_for_service("/link_attacher_node/attach")
    srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    if flag == "grabber":
        test_args = {
            "model_name_1": "ezrassor1",
            "link_name_1": "link6",
            "model_name_2": "paver{}".format(paver_count),
            "link_name_2": "paver_link",
        }
    else:
        test_args = {
            "model_name_1": "ezrassor1",
            "link_name_1": "base_link",
            "model_name_2": "paver{}".format(paver_count),
            "link_name_2": "paver_link",
        }
    srv(**test_args)


def DetachPaver(flag, paver_count):
    rospy.wait_for_service("/link_attacher_node/detach")
    srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    if flag == "grabber":
        test_args = {
            "model_name_1": "ezrassor1",
            "link_name_1": "link6",
            "model_name_2": "paver{}".format(paver_count),
            "link_name_2": "paver_link",
        }
    else:
        test_args = {
            "model_name_1": "ezrassor1",
            "link_name_1": "base_link",
            "model_name_2": "paver{}".format(paver_count),
            "link_name_2": "paver_link",
        }
    srv(**test_args)
