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

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import MagneticField


class MagConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("mag_converter_node")

        self.declare_parameter("au_to_tesla", 1.0)
        self.declare_parameter("noise_sigmas", [3.0e-07, 3.0e-07, 3.0e-07])
        self.declare_parameter("add_noise", True)
        self.declare_parameter("add_bias", True)
        self.declare_parameter("bias_sigmas", [1.0e-05, 1.0e-05, 1.0e-05])
        self.declare_parameter("input_topic", "MagnetometerSensor")
        self.declare_parameter("output_topic", "imu/mag_au")
        self.declare_parameter("bias_topic", "imu/mag/bias")
        self.declare_parameter("mag_frame", "imu_link")

        self.au_to_tesla = (
            self.get_parameter("au_to_tesla").get_parameter_value().double_value
        )
        self.noise_sigmas = (
            self.get_parameter("noise_sigmas").get_parameter_value().double_array_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )
        self.add_bias = self.get_parameter("add_bias").get_parameter_value().bool_value
        self.bias_sigmas = (
            self.get_parameter("bias_sigmas").get_parameter_value().double_array_value
        )
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        bias_topic = self.get_parameter("bias_topic").get_parameter_value().string_value
        self.mag_frame = (
            self.get_parameter("mag_frame").get_parameter_value().string_value
        )

        # Hard iron is static, so draw it once and hold it for the whole mission
        self.mag_bias = [
            random.gauss(0, sigma) if self.add_bias else 0.0
            for sigma in self.bias_sigmas
        ]

        self.subscription = self.create_subscription(
            MagneticField,
            input_topic,
            self.listener_callback,
            qos_profile_system_default,
        )
        # Reliable QoS to match SBG-SYSTEMS/sbg_ros2_driver
        self.publisher = self.create_publisher(
            MagneticField, output_topic, qos_profile_system_default
        )

        self.bias_publisher = self.create_publisher(
            MagneticField, bias_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def listener_callback(self, msg: MagneticField) -> None:
        msg.header.frame_id = self.mag_frame

        if self.add_bias:
            msg.magnetic_field.x += self.mag_bias[0]
            msg.magnetic_field.y += self.mag_bias[1]
            msg.magnetic_field.z += self.mag_bias[2]

        if self.add_noise:
            msg.magnetic_field.x += random.gauss(0, self.noise_sigmas[0])
            msg.magnetic_field.y += random.gauss(0, self.noise_sigmas[1])
            msg.magnetic_field.z += random.gauss(0, self.noise_sigmas[2])

        msg.magnetic_field_covariance[0] = self.noise_sigmas[0] ** 2
        msg.magnetic_field_covariance[4] = self.noise_sigmas[1] ** 2
        msg.magnetic_field_covariance[8] = self.noise_sigmas[2] ** 2

        if self.au_to_tesla != 1.0:
            msg.magnetic_field.x /= self.au_to_tesla
            msg.magnetic_field.y /= self.au_to_tesla
            msg.magnetic_field.z /= self.au_to_tesla

            for i in range(len(msg.magnetic_field_covariance)):
                msg.magnetic_field_covariance[i] /= self.au_to_tesla**2

        self.publisher.publish(msg)

        bias_msg = MagneticField()
        bias_msg.header = msg.header
        bias_msg.magnetic_field.x = self.mag_bias[0]
        bias_msg.magnetic_field.y = self.mag_bias[1]
        bias_msg.magnetic_field.z = self.mag_bias[2]

        self.bias_publisher.publish(bias_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    mag_converter_node = MagConverterNode()
    try:
        rclpy.spin(mag_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        mag_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
