"""Track the CPU and memory usage of all EZ-RASSOR processes.

Written by Samuel Lewis and Tiger Sachse.
"""
import rospy
import psutil
import std_msgs.msg


def start_node(
    default_node_name,
    trigger_string,
    cpu_usage_topic,
    memory_usage_topic,
    battery_usage_topic,
    queue_size):
    """Start the fun!"""
    try:
        rospy.init_node(default_node_name)
        cpu_usage_publisher = rospy.Publisher(
            cpu_usage_topic,
            std_msgs.msg.Float64,
            queue_size=queue_size,
        )
        memory_usage_publisher = rospy.Publisher(
            memory_usage_topic,
            std_msgs.msg.Float64,
            queue_size=queue_size,
        )
        battery_usage_publisher = rospy.Publisher(
            battery_usage_topic,
            std_msgs.msg.Float64,
            queue_size=queue_size,
        )

        # Publish the sum of CPU and memory usage for all processes whose calls
        # contain the trigger string.
        sleep_rate = rospy.Rate(1)
        cpu_count = psutil.cpu_count()
        while not rospy.is_shutdown():
            cpu_usage = 0.0
            memory_usage = 0.0
            for process in psutil.process_iter():
                try:
                    with process.oneshot():
                        for chunk in process.cmdline():
                            if trigger_string in chunk:
                                cpu_usage += process.cpu_percent()
                                memory_usage += process.memory_percent()
                                break
                except psutil.NoSuchProcess:
                    pass

            cpu_usage_publisher.publish(cpu_usage / cpu_count)
            memory_usage_publisher.publish(memory_usage)

            # Publish battery information as well.
            battery_usage = psutil.sensors_battery()
            if battery_usage is not None:
                battery_usage_publisher.publish(battery_usage.percent)

            # Don't do this too fast, lest you waste precious CPU cycles.
            sleep_rate.sleep()

    except rospy.ROSInterruptException:
        pass
