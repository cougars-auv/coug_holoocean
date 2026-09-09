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

import numpy as np
import rclpy
from dvl_msgs.msg import DVL, ConfigCommand, DVLBeam
from geometry_msgs.msg import TwistWithCovarianceStamped
from holoocean_interfaces.msg import DVLSensorRange
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

_FRD_R_FLU = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


class DvlConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("dvl_converter_node")

        self.declare_parameter("velocity_noise_sigmas", [0.02, 0.02, 0.02])
        self.declare_parameter("beam_velocity_noise_sigma", 0.011)
        self.declare_parameter("range_noise_sigma", 0.1)
        self.declare_parameter("beam0_frame", "beam0_link")
        self.declare_parameter("beam1_frame", "beam1_link")
        self.declare_parameter("beam2_frame", "beam2_link")
        self.declare_parameter("beam3_frame", "beam3_link")
        self.declare_parameter("max_range", 50.0)
        self.declare_parameter("add_noise", True)
        self.declare_parameter("velocity_input_topic", "DVLSensorVelocity")
        self.declare_parameter("range_input_topic", "DVLSensorRange")
        self.declare_parameter("output_topic", "dvl/data")
        self.declare_parameter("config_command_topic", "dvl/config/command")
        self.declare_parameter("dvl_frame", "dvl_link")

        self._velocity_noise_sigmas = self.get_parameter("velocity_noise_sigmas").value
        self._beam_velocity_noise_sigma = self.get_parameter("beam_velocity_noise_sigma").value
        self._range_noise_sigma = self.get_parameter("range_noise_sigma").value
        self._beam_frames = [
            self.get_parameter("beam0_frame").value,
            self.get_parameter("beam1_frame").value,
            self.get_parameter("beam2_frame").value,
            self.get_parameter("beam3_frame").value,
        ]
        self._max_range = self.get_parameter("max_range").value
        self._add_noise = self.get_parameter("add_noise").value
        velocity_input_topic = self.get_parameter("velocity_input_topic").value
        range_input_topic = self.get_parameter("range_input_topic").value
        output_topic = self.get_parameter("output_topic").value
        config_command_topic = self.get_parameter("config_command_topic").value
        self._dvl_frame = self.get_parameter("dvl_frame").value

        self._acoustic_enabled = True
        self._beam_ranges = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._input_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            velocity_input_topic,
            self._twist_callback,
            qos_profile_system_default,
        )
        self._range_sub = self.create_subscription(
            DVLSensorRange,
            range_input_topic,
            self._range_callback,
            qos_profile_system_default,
        )
        self._config_sub = self.create_subscription(
            ConfigCommand,
            config_command_topic,
            self._config_callback,
            qos_profile_sensor_data,
        )
        self._output_pub = self.create_publisher(DVL, output_topic, qos_profile_sensor_data)

        self.get_logger().info("Initialization complete.")

    def _range_callback(self, msg: DVLSensorRange) -> None:
        self._beam_ranges = msg.range

    def _config_callback(self, msg: ConfigCommand) -> None:
        if msg.command != "set_config" or msg.parameter_name != "acoustic_enabled":
            return

        self._acoustic_enabled = msg.parameter_value.strip().lower() == "true"
        self.get_logger().info(
            f"DVL acoustics {'enabled' if self._acoustic_enabled else 'disabled'}."
        )

    def _twist_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if not self._acoustic_enabled:
            return

        msg.header.frame_id = self._dvl_frame

        dvl_msg = DVL()
        dvl_msg.header.stamp = msg.header.stamp
        dvl_msg.header.frame_id = self._dvl_frame

        if self._add_noise:
            noise_x = random.gauss(0, self._velocity_noise_sigmas[0])
            noise_y = random.gauss(0, self._velocity_noise_sigmas[1])
            noise_z = random.gauss(0, self._velocity_noise_sigmas[2])
        else:
            noise_x = 0.0
            noise_y = 0.0
            noise_z = 0.0

        # Convert FLU -> FRD
        frd_velocity = _FRD_R_FLU.apply(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )

        dvl_msg.velocity.x = frd_velocity[0] + noise_x
        dvl_msg.velocity.y = frd_velocity[1] + noise_y
        dvl_msg.velocity.z = frd_velocity[2] + noise_z

        dvl_msg.velocity_valid = True

        # Convert nanoseconds to microseconds
        dvl_msg.time_of_validity = int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3)

        dvl_msg.covariance = [0.0] * 9
        dvl_msg.covariance[0] = self._velocity_noise_sigmas[0] ** 2
        dvl_msg.covariance[4] = self._velocity_noise_sigmas[1] ** 2
        dvl_msg.covariance[8] = self._velocity_noise_sigmas[2] ** 2

        dvl_msg.altitude = -1.0

        if self._beam_ranges is not None:
            frd_beam_axes = []
            for beam_frame in self._beam_frames:
                try:
                    dvl_T_beam_tf = self._tf_buffer.lookup_transform(
                        self._dvl_frame, beam_frame, rclpy.time.Time()
                    )
                except TransformException as e:
                    self.get_logger().warn(
                        f"Could not transform {self._dvl_frame} to {beam_frame}: {e}",
                        throttle_duration_sec=1.0,
                    )
                    break

                q = dvl_T_beam_tf.transform.rotation
                dvl_R_beam = Rotation.from_quat([q.x, q.y, q.z, q.w])

                # Each beam frame points +X down its own beam; convert FLU -> FRD
                frd_R_beam = _FRD_R_FLU * dvl_R_beam
                frd_beam_axes.append(frd_R_beam.apply([1.0, 0.0, 0.0]))

            if len(frd_beam_axes) == len(self._beam_frames):
                dvl_msg.beams = self._create_beam_msgs(
                    self._beam_ranges, frd_velocity, frd_beam_axes
                )

                beam_altitudes = [
                    beam.distance * frd_beam_axis[2]
                    for beam, frd_beam_axis in zip(dvl_msg.beams, frd_beam_axes, strict=True)
                    if beam.valid
                ]
                if beam_altitudes:
                    dvl_msg.altitude = sum(beam_altitudes) / len(beam_altitudes)

        self._output_pub.publish(dvl_msg)

    def _create_beam_msgs(
        self, ranges: np.ndarray, frd_velocity: np.ndarray, frd_beam_axes: list[np.ndarray]
    ) -> list[DVLBeam]:
        beams = []
        for beam_id, (beam_range, frd_beam_axis) in enumerate(
            zip(ranges, frd_beam_axes, strict=True)
        ):
            beam = DVLBeam()
            beam.id = beam_id
            beam.valid = bool(beam_range < self._max_range)

            if not beam.valid:
                beam.velocity = 0.0
                beam.distance = -1.0
            else:
                beam.velocity = float(frd_beam_axis @ frd_velocity)
                beam.distance = float(beam_range)

                if self._add_noise:
                    beam.velocity += random.gauss(0, self._beam_velocity_noise_sigma)
                    beam.distance += random.gauss(0, self._range_noise_sigma)

            beams.append(beam)
        return beams


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    dvl_converter_node = DvlConverterNode()
    try:
        rclpy.spin(dvl_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
