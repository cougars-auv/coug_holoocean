# Copyright (c) 2026 BYU FROST Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from holoocean_interfaces.msg import AgentCommand
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState


class FinStatePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("fin_state_publisher_node")

        self.declare_parameter("input_topic", "ControlCommand")
        self.declare_parameter("output_topic", "joint_states")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.joint_names = ["top_fin_joint", "left_fin_joint", "right_fin_joint"]

        self.input_sub = self.create_subscription(
            AgentCommand,
            input_topic,
            self.command_callback,
            qos_profile_system_default,
        )
        self.output_pub = self.create_publisher(
            JointState, output_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def command_callback(self, msg: AgentCommand) -> None:
        self.output_pub.publish(self.create_joint_state_msg(msg))

    def create_joint_state_msg(self, msg: AgentCommand) -> JointState:
        joint_state = JointState()
        joint_state.header.stamp = msg.header.stamp
        joint_state.name = self.joint_names

        rudder = -msg.command[0]
        starboard_elevator = msg.command[1]
        port_elevator = msg.command[2]
        joint_state.position = [rudder, port_elevator, starboard_elevator]
        return joint_state


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    fin_state_publisher_node = FinStatePublisherNode()
    try:
        rclpy.spin(fin_state_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        fin_state_publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
