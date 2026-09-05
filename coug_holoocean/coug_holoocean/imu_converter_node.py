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

import math
import random

import message_filters
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu


class ImuConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_converter_node")

        self.declare_parameter("sync_slop_sec", 0.05)
        self.declare_parameter("accel_noise_sigmas", [0.0079, 0.0079, 0.0079])
        self.declare_parameter("gyro_noise_sigmas", [0.00074, 0.00074, 0.00074])
        self.declare_parameter("ahrs_noise_sigmas", [0.00698, 0.00698, 0.01745])
        self.declare_parameter("add_noise", True)
        self.declare_parameter("add_bias", True)
        self.declare_parameter("accel_bias_rw_sigmas", [1.4e-5, 1.4e-5, 1.4e-5])
        self.declare_parameter("gyro_bias_rw_sigmas", [3.5e-6, 3.5e-6, 3.5e-6])
        self.declare_parameter("imu_input_topic", "IMUSensor")
        self.declare_parameter("ahrs_input_topic", "RotationSensor")
        self.declare_parameter("output_topic", "imu/data")
        self.declare_parameter("bias_topic", "imu/bias")
        self.declare_parameter("imu_frame", "imu_link")

        sync_slop_sec = self.get_parameter("sync_slop_sec").value
        self._accel_noise_sigmas = self.get_parameter("accel_noise_sigmas").value
        self._gyro_noise_sigmas = self.get_parameter("gyro_noise_sigmas").value
        self._ahrs_noise_sigmas = self.get_parameter("ahrs_noise_sigmas").value
        self._add_noise = self.get_parameter("add_noise").value
        self._add_bias = self.get_parameter("add_bias").value
        self._accel_bias_rw_sigmas = self.get_parameter("accel_bias_rw_sigmas").value
        self._gyro_bias_rw_sigmas = self.get_parameter("gyro_bias_rw_sigmas").value
        imu_input_topic = self.get_parameter("imu_input_topic").value
        ahrs_input_topic = self.get_parameter("ahrs_input_topic").value
        output_topic = self.get_parameter("output_topic").value
        bias_topic = self.get_parameter("bias_topic").value
        self._imu_frame = self.get_parameter("imu_frame").value

        self._accel_bias = [0.0, 0.0, 0.0]
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._last_stamp = None

        self._imu_sub = message_filters.Subscriber(
            self, Imu, imu_input_topic, qos_profile=qos_profile_system_default
        )
        self._ahrs_sub = message_filters.Subscriber(
            self,
            Vector3Stamped,
            ahrs_input_topic,
            qos_profile=qos_profile_system_default,
        )
        self._time_sync = message_filters.ApproximateTimeSynchronizer(
            [self._imu_sub, self._ahrs_sub], queue_size=10, slop=sync_slop_sec
        )
        self._time_sync.registerCallback(self._sync_callback)

        # Reliable QoS to match SBG-SYSTEMS/sbg_ros2_driver
        self._output_pub = self.create_publisher(
            Imu, output_topic, qos_profile_system_default
        )
        self._bias_pub = self.create_publisher(
            TwistWithCovarianceStamped, bias_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def _sync_callback(self, imu_msg: Imu, ahrs_msg: Vector3Stamped) -> None:
        roll_rad = math.radians(ahrs_msg.vector.x)
        pitch_rad = math.radians(ahrs_msg.vector.y)
        yaw_rad = math.radians(ahrs_msg.vector.z)

        map_R_ahrs = Rotation.from_euler("xyz", [roll_rad, pitch_rad, yaw_rad])

        if self._add_noise:
            # Perturb IMU orientation about the map-frame axes
            map_noise = [random.gauss(0, sigma) for sigma in self._ahrs_noise_sigmas]
            map_R_ahrs = Rotation.from_rotvec(map_noise) * map_R_ahrs

        q = map_R_ahrs.as_quat()

        imu_msg.header.frame_id = self._imu_frame

        if self._add_bias:
            current_stamp = (
                imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
            )
            if self._last_stamp is not None:
                dt = current_stamp - self._last_stamp
                if dt > 0.0:
                    sqrt_dt = math.sqrt(dt)
                    for i in range(3):
                        self._accel_bias[i] += random.gauss(
                            0, self._accel_bias_rw_sigmas[i] * sqrt_dt
                        )
                        self._gyro_bias[i] += random.gauss(
                            0, self._gyro_bias_rw_sigmas[i] * sqrt_dt
                        )
            self._last_stamp = current_stamp

            imu_msg.linear_acceleration.x += self._accel_bias[0]
            imu_msg.linear_acceleration.y += self._accel_bias[1]
            imu_msg.linear_acceleration.z += self._accel_bias[2]

            imu_msg.angular_velocity.x += self._gyro_bias[0]
            imu_msg.angular_velocity.y += self._gyro_bias[1]
            imu_msg.angular_velocity.z += self._gyro_bias[2]

        if self._add_noise:
            imu_msg.linear_acceleration.x += random.gauss(
                0, self._accel_noise_sigmas[0]
            )
            imu_msg.linear_acceleration.y += random.gauss(
                0, self._accel_noise_sigmas[1]
            )
            imu_msg.linear_acceleration.z += random.gauss(
                0, self._accel_noise_sigmas[2]
            )

            imu_msg.angular_velocity.x += random.gauss(0, self._gyro_noise_sigmas[0])
            imu_msg.angular_velocity.y += random.gauss(0, self._gyro_noise_sigmas[1])
            imu_msg.angular_velocity.z += random.gauss(0, self._gyro_noise_sigmas[2])

        imu_msg.linear_acceleration_covariance[0] = self._accel_noise_sigmas[0] ** 2
        imu_msg.linear_acceleration_covariance[4] = self._accel_noise_sigmas[1] ** 2
        imu_msg.linear_acceleration_covariance[8] = self._accel_noise_sigmas[2] ** 2

        imu_msg.angular_velocity_covariance[0] = self._gyro_noise_sigmas[0] ** 2
        imu_msg.angular_velocity_covariance[4] = self._gyro_noise_sigmas[1] ** 2
        imu_msg.angular_velocity_covariance[8] = self._gyro_noise_sigmas[2] ** 2

        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        imu_msg.orientation_covariance[0] = self._ahrs_noise_sigmas[0] ** 2
        imu_msg.orientation_covariance[4] = self._ahrs_noise_sigmas[1] ** 2
        imu_msg.orientation_covariance[8] = self._ahrs_noise_sigmas[2] ** 2

        self._output_pub.publish(imu_msg)

        bias_msg = TwistWithCovarianceStamped()
        bias_msg.header = imu_msg.header
        bias_msg.twist.twist.linear.x = self._accel_bias[0]
        bias_msg.twist.twist.linear.y = self._accel_bias[1]
        bias_msg.twist.twist.linear.z = self._accel_bias[2]
        bias_msg.twist.twist.angular.x = self._gyro_bias[0]
        bias_msg.twist.twist.angular.y = self._gyro_bias[1]
        bias_msg.twist.twist.angular.z = self._gyro_bias[2]

        self._bias_pub.publish(bias_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    imu_converter_node = ImuConverterNode()
    try:
        rclpy.spin(imu_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
