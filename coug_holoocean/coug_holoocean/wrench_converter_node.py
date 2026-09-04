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

import math

import rclpy
from geometry_msgs.msg import WrenchStamped
from holoocean_interfaces.msg import AgentCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

# actuator.py
RHO = 1026.0
D_PROP = 0.14
T_PROP = 0.1
KT_0 = 0.4566
KT_MAX = 0.1798
JA_MAX = 0.6632
W = 0.056

C1 = (1.0 - T_PROP) * RHO * pow(D_PROP, 4) * KT_0
C2 = (
    (1.0 - T_PROP)
    * RHO
    * pow(D_PROP, 4)
    * (KT_MAX - KT_0)
    / JA_MAX
    * ((1 - W) / D_PROP)
)


class WrenchConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("wrench_converter_node")

        self.declare_parameter("control_topic", "ControlCommand")
        self.declare_parameter("odom_topic", "DynamicsSensorOdom")
        self.declare_parameter("wrench_raw_topic", "cmd_wrench_raw")
        self.declare_parameter("wrench_topic", "cmd_wrench")
        self.declare_parameter("wrench_frame", "com_link")

        control_topic = self.get_parameter("control_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        wrench_raw_topic = self.get_parameter("wrench_raw_topic").value
        wrench_topic = self.get_parameter("wrench_topic").value
        self._wrench_frame = self.get_parameter("wrench_frame").value

        self._control_sub = self.create_subscription(
            AgentCommand,
            control_topic,
            self._control_callback,
            qos_profile_system_default,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile_system_default,
        )
        self._wrench_raw_pub = self.create_publisher(
            WrenchStamped, wrench_raw_topic, qos_profile_system_default
        )
        self._wrench_pub = self.create_publisher(
            WrenchStamped, wrench_topic, qos_profile_system_default
        )

        self._speed = 0.0

        self.get_logger().info("Initialization complete.")

    def control_callback(self, msg: AgentCommand) -> None:
        thruster_rpm = msg.command[-1]

        n_rps = thruster_rpm / 60.0

        # Assuming no spool up/down delays
        force_x_raw = C1 * abs(n_rps) * n_rps
        force_x = force_x_raw + C2 * n_rps * self._speed if n_rps > 0 else force_x_raw

        raw_wrench_msg = WrenchStamped()
        raw_wrench_msg.header.stamp = msg.header.stamp
        raw_wrench_msg.header.frame_id = self._wrench_frame
        raw_wrench_msg.wrench.force.x = force_x_raw
        self._wrench_raw_pub.publish(raw_wrench_msg)

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = msg.header.stamp
        wrench_msg.header.frame_id = self._wrench_frame
        wrench_msg.wrench.force.x = force_x
        self._wrench_pub.publish(wrench_msg)

    def odom_callback(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        self._speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    wrench_converter_node = WrenchConverterNode()
    try:
        rclpy.spin(wrench_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        wrench_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
