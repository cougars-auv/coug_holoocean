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

import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

_UNKNOWN_COVARIANCE = -1.0
_UNMEASURED_VARIANCE = 1e9


class DepthConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_converter_node")

        self.declare_parameter("position_z_noise_sigma", 0.02)
        self.declare_parameter("add_noise", True)
        self.declare_parameter("input_topic", "DepthSensor")
        self.declare_parameter("output_topic", "depth/odometry")
        self.declare_parameter("depth_frame", "depth_link")
        self.declare_parameter("map_frame", "map")

        self._position_z_noise_sigma = self.get_parameter("position_z_noise_sigma").value
        self._add_noise = self.get_parameter("add_noise").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._depth_frame = self.get_parameter("depth_frame").value
        self._map_frame = self.get_parameter("map_frame").value

        self._input_sub = self.create_subscription(
            Odometry, input_topic, self._odom_callback, qos_profile_system_default
        )
        self._output_pub = self.create_publisher(Odometry, output_topic, qos_profile_system_default)

        self.get_logger().info("Initialization complete.")

    def _odom_callback(self, msg: Odometry) -> None:
        depth_msg = Odometry()
        depth_msg.header.stamp = msg.header.stamp
        depth_msg.header.frame_id = self._map_frame
        depth_msg.child_frame_id = self._depth_frame

        depth = msg.pose.pose.position.z
        if self._add_noise:
            depth += random.gauss(0, self._position_z_noise_sigma)
        depth_msg.pose.pose.position.z = depth

        depth_msg.pose.covariance[14] = self._position_z_noise_sigma**2

        depth_msg.pose.covariance[0] = _UNMEASURED_VARIANCE
        depth_msg.pose.covariance[7] = _UNMEASURED_VARIANCE
        depth_msg.pose.covariance[21] = _UNMEASURED_VARIANCE
        depth_msg.pose.covariance[28] = _UNMEASURED_VARIANCE
        depth_msg.pose.covariance[35] = _UNMEASURED_VARIANCE
        depth_msg.twist.covariance[0] = _UNKNOWN_COVARIANCE

        self._output_pub.publish(depth_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    depth_converter_node = DepthConverterNode()
    try:
        rclpy.spin(depth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
