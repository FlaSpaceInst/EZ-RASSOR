"""Track the CPU usage of all EZ-RASSOR processes.

This node presents the total CPU usage in a way that can be understood by the
ezrassor_dashboard. It separates the CPU consumption of Gazebo from the processes
being run in ROS, which allows researchers to better understand the power drain
and CPU demands of any autonomous systems they are testing.

Written by Samuel Lewis and Tiger Sachse.
"""
import rospy
import numpy
import subprocess
import std_msgs.msg
import multiprocessing


def get_pids(process_names):
    """"""
    pids = []
    for process_name in process_names:
        returned_pids = subprocess.check_output(
            "pgrep -f {0}".format(process_name),
            shell=True,
        )

        if returned_pids is not None:
            pids.extend((int(pid) for pid in returned_pids.split("\n"))) #

    return pids


def get_cpu_usage(pids):
    """"""
    cpu_usage = 0.0
    for pid in pids:
        unparsed_cpu_usage = subprocess.check_output(
            "top -b -n 1 -p {0}".format(pid),
            shell=True,
        )

        cpu_usage_line_items = unparsed_cpu_usage[-1].split()
        try:
            cpu_usage += float(cpu_usage_line_items[8])
        except ValueError:
            pass

    return cpu_usage


def start_node(default_node_name, process_names, cpu_usage_topic, queue_size):
    """"""
    try:
        rospy.init_node(default_node_name, anonymous=True)
        cpu_usage_publisher = rospy.Publisher(
            cpu_usage_topic,
            std_msgs.msg.Float64,
            queue_size=queue_size,
        )

        while not rospy.is_shutdown():
            process_pids = get_pids(process_names)
            cpu_usage_publisher.publish(get_cpu_usage(process_pids))
    except rospy.ROSInterruptException:
        pass
