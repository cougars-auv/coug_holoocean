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


class WrenchConverterNode(Node):
    """
    ROS 2 node that converts HoloOcean AgentCommand messages to WrenchStamped messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("wrench_converter_node")

        self.declare_parameter("control_topic", "ControlCommand")
        self.declare_parameter("odom_topic", "DynamicsSensorOdom")
        self.declare_parameter("wrench_raw_topic", "cmd_wrench_raw")
        self.declare_parameter("wrench_topic", "cmd_wrench")
        self.declare_parameter("wrench_frame", "com_link")

        control_topic = (
            self.get_parameter("control_topic").get_parameter_value().string_value
        )
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        wrench_raw_topic = (
            self.get_parameter("wrench_raw_topic").get_parameter_value().string_value
        )
        wrench_topic = (
            self.get_parameter("wrench_topic").get_parameter_value().string_value
        )
        self.wrench_frame = (
            self.get_parameter("wrench_frame").get_parameter_value().string_value
        )

        self.control_sub = self.create_subscription(
            AgentCommand,
            control_topic,
            self.control_callback,
            qos_profile_system_default,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos_profile_system_default,
        )
        self.wrench_raw_pub = self.create_publisher(
            WrenchStamped, wrench_raw_topic, qos_profile_system_default
        )
        self.wrench_pub = self.create_publisher(
            WrenchStamped, wrench_topic, qos_profile_system_default
        )

        # HoloOcean CougUV Thrusters (actuator.py)
        self.rho = 1026.0
        self.d_prop = 0.14
        self.t_prop = 0.1
        self.kt_0 = 0.4566
        self.kt_max = 0.1798
        self.ja_max = 0.6632
        self.w = 0.056

        self.c1 = (1.0 - self.t_prop) * self.rho * pow(self.d_prop, 4) * self.kt_0
        self.c2 = (
            (1.0 - self.t_prop)
            * self.rho
            * pow(self.d_prop, 4)
            * (self.kt_max - self.kt_0)
            / self.ja_max
            * ((1 - self.w) / self.d_prop)
        )

        self.speed = 0.0

        self.get_logger().info("Initialization complete.")

    def control_callback(self, msg: AgentCommand) -> None:
        """
        Convert a CougUV command into a COM-frame wrench and publish it.

        :param msg: AgentCommand message containing control surface/thruster values.
        """
        thruster_rpm = msg.command[-1]

        n_rps = thruster_rpm / 60.0

        # IMPORTANT! Assuming no spool up/down delays
        # This only matters under the 'stepInput' and 'manualControl' modes, which
        # report the raw RPM command. 'depthHeadingAutopilot' reports the propeller
        # state after actuation, so the T_n spool lag is already accounted for.
        force_x_raw = self.c1 * abs(n_rps) * n_rps
        if n_rps > 0:
            force_x = force_x_raw + self.c2 * n_rps * self.speed
        else:
            force_x = force_x_raw

        raw_wrench_msg = WrenchStamped()
        raw_wrench_msg.header.stamp = msg.header.stamp
        raw_wrench_msg.header.frame_id = self.wrench_frame
        raw_wrench_msg.wrench.force.x = force_x_raw
        self.wrench_raw_pub.publish(raw_wrench_msg)

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = msg.header.stamp
        wrench_msg.header.frame_id = self.wrench_frame
        wrench_msg.wrench.force.x = force_x
        self.wrench_pub.publish(wrench_msg)

    def odom_callback(self, msg: Odometry) -> None:
        """
        Store the vehicle speed for CougUV command processing.

        :param msg: Odometry message containing the current COM velocity.
        """
        linear = msg.twist.twist.linear
        self.speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)


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
