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
from enum import StrEnum

import rclpy
from geometry_msgs.msg import TwistStamped
from holoocean_interfaces.msg import AgentCommand
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class AgentType(StrEnum):
    BLUEROV2 = "bluerov2"
    SURFACE_VESSEL = "surface_vessel"


# BlueROV2.h/BlueROV2.cpp
BR_LINEAR_DRAG = 11.5  # mass(11.5) × SetLinearDamping(1.0)
BR_ANGULAR_DRAG = 0.225  # Iz(0.3) × SetAngularDamping(0.75)
BR_MAX_THRUST = 28.75  # BR_MAX_THRUST
BR_VERT_X, BR_VERT_Y = 0.12, 0.2181  # 'thrusterLocations' 0-3, from COM (m)
BR_ANGLED_X, BR_ANGLED_Y = 0.1562, 0.0988  # 'thrusterLocations' 4-7 (m)

# Force/torque to hold steady state against drag at unit velocity.
BR_H_SCALE = BR_LINEAR_DRAG / (4.0 * math.sqrt(0.5))
BR_V_SCALE = BR_LINEAR_DRAG / 4.0
BR_R_SCALE = BR_ANGULAR_DRAG / (4.0 * BR_VERT_Y)
BR_P_SCALE = BR_ANGULAR_DRAG / (4.0 * BR_VERT_X)
BR_Y_SCALE = BR_ANGULAR_DRAG / (4.0 * (BR_ANGLED_X + BR_ANGLED_Y) * math.sqrt(0.5))

# SurfaceVessel.h/SurfaceVessel.cpp
SV_LINEAR_DRAG = 600.0  # mass(200) × SetLinearDamping(3.0)
SV_ANGULAR_DRAG = 384.5  # Iz(512.7) × SetAngularDamping(0.75)
SV_MAX_THRUST = 1500.0  # SV_MAX_THRUST
SV_THRUSTER_Y = 1.0  # 'thrusterLocations' half-separation (m)

SV_H_SCALE = SV_LINEAR_DRAG
SV_Y_SCALE = SV_ANGULAR_DRAG


class CmdVelConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_converter_node")

        self.declare_parameter("agent_name", "auv0")
        self.declare_parameter("agent_type", AgentType.BLUEROV2)
        self.declare_parameter("input_topic", "cmd_vel_out")
        self.declare_parameter("output_topic", "/command/agent")

        self.agent_name = self.get_parameter("agent_name").value
        agent_type = self.get_parameter("agent_type").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        try:
            self.agent_type = AgentType(agent_type)
        except ValueError as error:
            raise ValueError(
                f"Unknown agent_type '{agent_type}' "
                f"(expected '{AgentType.BLUEROV2}' or '{AgentType.SURFACE_VESSEL}')"
            ) from error

        self.input_sub = self.create_subscription(
            TwistStamped,
            input_topic,
            self.twist_callback,
            qos_profile_system_default,
        )
        self.output_pub = self.create_publisher(
            AgentCommand, output_topic, qos_profile_system_default
        )

        if self.agent_type == AgentType.BLUEROV2:
            self.thruster_limit = BR_MAX_THRUST
            self.h_scale = BR_H_SCALE
            self.v_scale = BR_V_SCALE
            self.r_scale = BR_R_SCALE
            self.p_scale = BR_P_SCALE
            self.y_scale = BR_Y_SCALE
        else:
            self.thruster_limit = SV_MAX_THRUST
            self.thruster_y = SV_THRUSTER_Y
            self.h_scale = SV_H_SCALE
            self.y_scale = SV_Y_SCALE

        self.get_logger().info("Initialization complete.")

    def twist_callback(self, msg: TwistStamped) -> None:
        self.output_pub.publish(self.create_agent_command_msg(msg))

    def create_agent_command_msg(self, msg: TwistStamped) -> AgentCommand:
        agent_cmd = AgentCommand()
        agent_cmd.header.stamp = self.get_clock().now().to_msg()
        agent_cmd.header.frame_id = self.agent_name

        if self.agent_type == AgentType.BLUEROV2:
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
        return agent_cmd

    def bluerov2_command(self, msg: TwistStamped) -> list[float]:
        # Assuming no quadratic drag
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
        # Assuming no quadratic drag
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
