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
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TwistStamped
from holoocean_interfaces.msg import AgentCommand

BLUEROV2 = "bluerov2"
SURFACE_VESSEL = "surface_vessel"


class CmdVelConverterNode(Node):
    """
    ROS 2 node that converts TwistStamped messages to HoloOcean AgentCommand messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_converter_node")

        self.declare_parameter("agent_name", "auv0")
        self.declare_parameter("agent_type", BLUEROV2)
        self.declare_parameter("input_topic", "cmd_vel_out")
        self.declare_parameter("output_topic", "/command/agent")

        self.agent_name = (
            self.get_parameter("agent_name").get_parameter_value().string_value
        )
        self.agent_type = (
            self.get_parameter("agent_type").get_parameter_value().string_value
        )
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        if self.agent_type not in (BLUEROV2, SURFACE_VESSEL):
            raise ValueError(
                f"Unknown agent_type '{self.agent_type}' "
                f"(expected '{BLUEROV2}' or '{SURFACE_VESSEL}')"
            )

        self.subscription = self.create_subscription(
            TwistStamped,
            input_topic,
            self.listener_callback,
            qos_profile_system_default,
        )
        self.publisher = self.create_publisher(
            AgentCommand, output_topic, qos_profile_system_default
        )

        if self.agent_type == BLUEROV2:
            # BlueROV2.h/BlueROV2.cpp
            linear_drag = 11.5  # mass(11.5) × SetLinearDamping(1.0)
            angular_drag = 0.225  # Iz(0.3) × SetAngularDamping(0.75)
            self.thruster_limit = 28.75  # BR_MAX_THRUST
            vert_x, vert_y = 0.12, 0.2181  # 'thrusterLocations' 0-3, from COM (m)
            angled_x, angled_y = 0.1562, 0.0988  # 'thrusterLocations' 4-7 (m)

            # Force/torque to hold steady state against drag at unit velocity.
            self.h_scale = linear_drag / (4.0 * math.sqrt(0.5))
            self.v_scale = linear_drag / 4.0
            self.r_scale = angular_drag / (4.0 * vert_y)
            self.p_scale = angular_drag / (4.0 * vert_x)
            self.y_scale = angular_drag / (4.0 * (angled_x + angled_y) * math.sqrt(0.5))
        else:
            # SurfaceVessel.h/SurfaceVessel.cpp
            linear_drag = 600.0  # mass(200) × SetLinearDamping(3.0)
            angular_drag = 384.5  # Iz(512.7) × SetAngularDamping(0.75)
            self.thruster_limit = 1500.0  # SV_MAX_THRUST
            self.thruster_y = 1.0  # 'thrusterLocations' half-separation (m)

            self.h_scale = linear_drag
            self.y_scale = angular_drag

        self.get_logger().info("Initialization complete.")

    def listener_callback(self, msg: TwistStamped) -> None:
        """
        Convert a commanded velocity into a HoloOcean-compatible command and publish it.

        :param msg: TwistStamped message containing linear and angular velocities.
        """
        agent_cmd = AgentCommand()
        agent_cmd.header.stamp = self.get_clock().now().to_msg()
        agent_cmd.header.frame_id = self.agent_name

        if self.agent_type == BLUEROV2:
            raw_cmds = self.bluerov2_command(msg)
        else:
            raw_cmds = self.surface_vessel_command(msg)

        max_req = max([abs(x) for x in raw_cmds])
        if max_req > self.thruster_limit:
            scale_factor = self.thruster_limit / max_req
            final_cmds = [x * scale_factor for x in raw_cmds]
        else:
            final_cmds = raw_cmds

        agent_cmd.command = final_cmds
        self.publisher.publish(agent_cmd)

    def bluerov2_command(self, msg: TwistStamped) -> list[float]:
        """
        Map a commanded velocity onto the eight BlueROV2 thrusters.

        :param msg: TwistStamped message containing linear and angular velocities.
        :return: Thruster forces in the HoloOcean command order.
        """
        # IMPORTANT! Assuming no quadratic drag
        fwd = msg.twist.linear.x * self.h_scale
        lat = msg.twist.linear.y * self.h_scale
        vert = msg.twist.linear.z * self.v_scale
        roll = msg.twist.angular.x * self.r_scale
        pitch = msg.twist.angular.y * self.p_scale
        yaw = msg.twist.angular.z * self.y_scale

        cmd_0 = vert - pitch - roll
        cmd_1 = vert - pitch + roll
        cmd_2 = vert + pitch + roll
        cmd_3 = vert + pitch - roll
        cmd_4 = fwd + lat + yaw
        cmd_5 = fwd - lat - yaw
        cmd_6 = fwd + lat - yaw
        cmd_7 = fwd - lat + yaw

        return [cmd_0, cmd_1, cmd_2, cmd_3, cmd_4, cmd_5, cmd_6, cmd_7]

    def surface_vessel_command(self, msg: TwistStamped) -> list[float]:
        """
        Map a commanded velocity onto the two SurfaceVessel thrusters.

        :param msg: TwistStamped message containing linear and angular velocities.
        :return: Left and right thruster forces.
        """
        # IMPORTANT! Assuming no quadratic drag
        fwd = msg.twist.linear.x * self.h_scale
        yaw = msg.twist.angular.z * self.y_scale

        cmd_left = fwd / 2.0 - yaw / (2.0 * self.thruster_y)
        cmd_right = fwd / 2.0 + yaw / (2.0 * self.thruster_y)

        return [cmd_left, cmd_right]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    cmd_vel_converter = CmdVelConverterNode()
    try:
        rclpy.spin(cmd_vel_converter)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_converter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
