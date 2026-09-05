# Copyright 2026 BYU FROST Lab
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
from coug_interfaces.msg import ControlSetpoint
from holoocean_interfaces.msg import DesiredCommand
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header

MIN_SPEED_RPM = 0.0
MAX_SPEED_RPM = 1525.0


class HsdConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("hsd_converter_node")

        self.declare_parameter("agent_name", "auv0")
        self.declare_parameter("hsd_topic", "cmd_hsd")
        self.declare_parameter("output_heading_topic", "/heading")
        self.declare_parameter("output_speed_topic", "/speed")
        self.declare_parameter("output_depth_topic", "/depth")

        self._agent_name = self.get_parameter("agent_name").value
        self._hsd_topic = self.get_parameter("hsd_topic").value
        self._output_heading_topic = self.get_parameter("output_heading_topic").value
        self._output_speed_topic = self.get_parameter("output_speed_topic").value
        self._output_depth_topic = self.get_parameter("output_depth_topic").value

        self._output_heading_pub = self.create_publisher(
            DesiredCommand, self._output_heading_topic, qos_profile_system_default
        )
        self._output_speed_pub = self.create_publisher(
            DesiredCommand, self._output_speed_topic, qos_profile_system_default
        )
        self._output_depth_pub = self.create_publisher(
            DesiredCommand, self._output_depth_topic, qos_profile_system_default
        )

        self._hsd_sub = self.create_subscription(
            ControlSetpoint,
            self._hsd_topic,
            self._hsd_callback,
            qos_profile_system_default,
        )

        self.get_logger().info("Initialization complete.")

    def _hsd_callback(self, msg: ControlSetpoint) -> None:
        self._output_heading_pub.publish(self._create_desired_command_msg(msg.heading))
        self._output_speed_pub.publish(
            self._create_desired_command_msg(
                max(MIN_SPEED_RPM, min(MAX_SPEED_RPM, msg.speed_rpm))
            )
        )
        self._output_depth_pub.publish(
            self._create_desired_command_msg(max(-msg.depth, 0.0))
        )

    def _create_desired_command_msg(self, value: float) -> DesiredCommand:
        msg = DesiredCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._agent_name
        msg.data = float(value)
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HsdConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
