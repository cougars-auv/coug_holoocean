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

_MIN_SPEED_RPM = -1525.0
_MAX_SPEED_RPM = 1525.0


class HsdConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("hsd_converter_node")

        self.declare_parameter("agent_name", "auv0")
        self.declare_parameter("input_topic", "cmd_hsd")
        self.declare_parameter("heading_output_topic", "/heading")
        self.declare_parameter("speed_output_topic", "/speed")
        self.declare_parameter("depth_output_topic", "/depth")

        self._agent_name = self.get_parameter("agent_name").value
        input_topic = self.get_parameter("input_topic").value
        heading_output_topic = self.get_parameter("heading_output_topic").value
        speed_output_topic = self.get_parameter("speed_output_topic").value
        depth_output_topic = self.get_parameter("depth_output_topic").value

        self._input_sub = self.create_subscription(
            ControlSetpoint,
            input_topic,
            self._hsd_callback,
            qos_profile_system_default,
        )
        self._heading_pub = self.create_publisher(
            DesiredCommand, heading_output_topic, qos_profile_system_default
        )
        self._speed_pub = self.create_publisher(
            DesiredCommand, speed_output_topic, qos_profile_system_default
        )
        self._depth_pub = self.create_publisher(
            DesiredCommand, depth_output_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def _hsd_callback(self, msg: ControlSetpoint) -> None:
        self._heading_pub.publish(self._convert_to_desired_command(msg.heading))
        self._speed_pub.publish(
            self._convert_to_desired_command(
                max(_MIN_SPEED_RPM, min(_MAX_SPEED_RPM, msg.speed_rpm))
            )
        )
        self._depth_pub.publish(self._convert_to_desired_command(max(-msg.depth, 0.0)))

    def _convert_to_desired_command(self, value: float) -> DesiredCommand:
        msg = DesiredCommand()
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
