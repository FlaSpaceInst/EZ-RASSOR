"""Listen for POST requests and route data to the appropriate topics.

Written by Tiger Sachse.
Inspired by Camilo Lozano.
"""
import time
import json
import rospy
import threading
import std_msgs.msg
import BaseHTTPServer
import geometry_msgs.msg


def get_custom_handler(publishers):
    """Get a custom HTTP request handler with appropriate publishers."""
    class CustomRequestHandler(BaseHTTPServer.BaseHTTPRequestHandler):
        """Handle JSON POST requests on a simple HTTP server."""
        target_coordinates_publisher = publishers[0]
        autonomous_toggles_publisher = publishers[1]
        wheel_instructions_publisher = publishers[2]
        front_arm_instructions_publisher = publishers[3]
        back_arm_instructions_publisher = publishers[4]
        front_drum_instructions_publisher = publishers[5]
        back_drum_instructions_publisher = publishers[6]

        def do_POST(self):
            """Handle all POST requests."""
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            content_length = int(self.headers.getheader("content-length", 0))
            instructions = json.loads(
                self.rfile.read(
                    content_length,
                ),
            )
            self.wfile.write(json.dumps({"status" : 200}))
            self._publish_instructions(instructions)

        def _publish_instructions(self, instructions):
            """Publish instructions to their respective topics."""
            if "autonomous_toggles" in instructions:
                self.autonomous_toggles_publisher.publish(
                    int(instructions["autonomous_toggles"]),
                )

            if "target_coordinate" in instructions:
                target_coordinate = geometry_msgs.msg.Point()
                target_coordinate.x = float(instructions["target_coordinate"]["x"])
                target_coordinate.y = float(instructions["target_coordinate"]["y"])
                self.target_coordinates_publisher.publish(target_coordinate)

            if "wheel_instruction" in instructions:
                wheel_instruction = geometry_msgs.msg.Twist()
                if instructions["wheel_instruction"] == "forward":
                    wheel_instruction.linear.x = 1.0
                elif instructions["wheel_instruction"] == "backward":
                    wheel_instruction.linear.x = -1.0
                elif instructions["wheel_instruction"] == "left":
                    wheel_instruction.angular.z = 1.0
                elif instructions["wheel_instruction"] == "right":
                    wheel_instruction.angular.z = -1.0
                self.wheel_instructions_publisher.publish(wheel_instruction)

            self._check_float_instruction(
                "front_arm_instruction",
                instructions,
                self.front_arm_instructions_publisher,
            )
            self._check_float_instruction(
                "back_arm_instruction",
                instructions,
                self.back_arm_instructions_publisher,
            )
            self._check_float_instruction(
                "front_drum_instruction",
                instructions,
                self.front_drum_instructions_publisher,
            )
            self._check_float_instruction(
                "back_drum_instruction",
                instructions,
                self.back_drum_instructions_publisher,
            )

        def _check_float_instruction(
            self,
            instruction,
            instructions,
            instruction_publisher):
            """Handle an up/down float instruction."""
            if instruction in instructions:
                if instructions[instruction] == "up":
                    instruction_publisher.publish(1.0)
                elif instructions[instruction] == "down":
                    instruction_publisher.publish(-1.0)

    return CustomRequestHandler


def kill_server(server, sleep_duration):
    """Wait for roscore to die, then kill the server."""
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

        # Load ROS params into the script.
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

        # Create a whole heap of publishers.
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
            get_custom_handler((
                target_coordinates_publisher,
                autonomous_toggles_publisher,
                wheel_instructions_publisher,
                front_arm_instructions_publisher,
                back_arm_instructions_publisher,
                front_drum_instructions_publisher,
                back_drum_instructions_publisher,
            )),
        )

        # Launch a kill thread that kills the server when roscore dies.
        kill_thread = threading.Thread(
            target=kill_server,
            args=(
                server,
                sleep_duration,
            ),
        )
        kill_thread.start()
        
        # Run the server infinitely.
        server.serve_forever(poll_interval=sleep_duration)

    # If anything goes wrong, make sure to shut down the server.
    except Exception:
        server.shutdown()
