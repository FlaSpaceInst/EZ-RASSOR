"""Listen for and route POST requests to the ROS graph's requests topic.

Written by Tiger Sachse and Camilo Lozano.
"""
import time
import json
import rospy
import threading
import std_msgs.msg
import BaseHTTPServer
import geometry_msgs.msg

def get_custom_handler(
    target_coordinates_publisher,
    autonomous_toggles_publisher,
    wheel_instructions_publisher,
    front_arm_instructions_publisher,
    back_arm_instructions_publisher,
    front_drum_instructions_publisher,
    back_drum_instructions_publisher):
    """"""

    class CustomRequestHandler(BaseHTTPServer.BaseHTTPRequestHandler):
        """Handle POST requests on a simple HTTP server."""
        #target_coordinates_publisher = target_coordinates_publisher
        #autonomous_toggles_publisher = autonomous_toggles_publisher
        #wheel_instructions_publisher = wheel_instructions_publisher
        #front_arm_instructions_publisher = front_arm_instructions_publisher
        #back_arm_instructions_publisher = back_arm_instructions_publisher
        #front_drum_instructions_publisher = front_drum_instructions_publisher
        #back_drum_instructions_publisher = back_drum_instructions_publisher

        def do_POST(self):
            """Handle all POST requests."""
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()

            content_length = int(self.headers.getheader("content-length", 0))
            body = json.loads(
                self.rfile.read(
                    content_length,
                ),
            )
            print(body)
            #self.publisher.publish(int(body))
            self.wfile.write(json.dumps({"status" : 200}))

    return CustomRequestHandler


def kill_server(server, sleep_duration):
    """"""
    while not rospy.core.is_shutdown():
        rospy.sleep(sleep_duration)
    server.shutdown()


def start_node(
    default_node_name,
    target_coordinates_topic,
    autonomous_toggles_topic,
    queue_size,
    sleep_duration):
    """Start the node and let the fun begin!"""
    try:
        rospy.init_node(default_node_name)

        port = rospy.get_param(rospy.get_name() + "/port")
        wheel_instructions_topic = rospy.get_param(
            rospy.get_name() + "/wheel_instructions_topic",
        )
        front_arm_instructions_topic = rospy.get_param(
            rospy.get_name() + "/front_arm_instructions_topic",
        )
        back_arm_instructions_topic = rospy.get_param(
            rospy.get_name() + "/back_arm_instructions_topic",
        )
        front_drum_instructions_topic = rospy.get_param(
            rospy.get_name() + "/front_drum_instructions_topic",
        )
        back_drum_instructions_topic = rospy.get_param(
            rospy.get_name() + "/back_drum_instructions_topic",
        )

        target_coordinates_publisher = rospy.Publisher(
            target_coordinates_topic,
            geometry_msgs.msg.Point,
            queue_size=queue_size,
        )
        autonomous_toggles_publisher = rospy.Publisher(
            autonomous_toggles_topic,
            std_msgs.msg.Int8,
            queue_size=queue_size,
        )
        wheel_instructions_publisher = rospy.Publisher(
            wheel_instructions_topic,
            geometry_msgs.msg.Twist,
            queue_size=queue_size,
        )
        front_arm_instructions_publisher = rospy.Publisher(
            front_arm_instructions_topic,
            std_msgs.msg.Float32,
            queue_size=queue_size,
        )
        back_arm_instructions_publisher = rospy.Publisher(
            back_arm_instructions_topic,
            std_msgs.msg.Float32,
            queue_size=queue_size,
        )
        front_drum_instructions_publisher = rospy.Publisher(
            front_drum_instructions_topic,
            std_msgs.msg.Float32,
            queue_size=queue_size,
        )
        back_drum_instructions_publisher = rospy.Publisher(
            back_drum_instructions_topic,
            std_msgs.msg.Float32,
            queue_size=queue_size,
        )
        
        # Create an HTTP server.
        server = BaseHTTPServer.HTTPServer(
            ("", port),
            get_custom_handler(
                target_coordinates_publisher,
                autonomous_toggles_publisher,
                wheel_instructions_publisher,
                front_arm_instructions_publisher,
                back_arm_instructions_publisher,
                front_drum_instructions_publisher,
                back_drum_instructions_publisher,
            ),
        )

        # 
        kill_thread = threading.Thread(
            target=kill_server,
            args=(
                server,
                sleep_duration,
            ),
        )
        kill_thread.start()

        server.serve_forever(poll_interval=sleep_duration)

    except Exception:
        server.shutdown()
