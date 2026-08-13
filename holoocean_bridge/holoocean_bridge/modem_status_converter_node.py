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

import message_filters
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from scipy.spatial.transform import Rotation
from seatrac_interfaces.msg import ModemStatus
from sensor_msgs.msg import Imu

from holoocean_bridge.utils import seatrac_enums as seatrac

_NED_R_ENU = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]).inv()
_FLU_R_FRD = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


class ModemStatusConverterNode(Node):
    """
    ROS 2 node that converts Imu and Odometry messages to ModemStatus messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("modem_status_converter_node")

        self.declare_parameter("sync_slop_sec", 0.05)
        self.declare_parameter("imu_input_topic", "modem/imu/data")
        self.declare_parameter("depth_input_topic", "modem/depth/odometry")
        self.declare_parameter("output_topic", "modem_status")

        sync_slop_sec = self.get_parameter("sync_slop_sec").value
        imu_input_topic = self.get_parameter("imu_input_topic").value
        depth_input_topic = self.get_parameter("depth_input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.start_time = self.get_clock().now()

        self.imu_sub = message_filters.Subscriber(
            self, Imu, imu_input_topic, qos_profile=qos_profile_system_default
        )
        self.depth_sub = message_filters.Subscriber(
            self, Odometry, depth_input_topic, qos_profile=qos_profile_system_default
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.depth_sub], queue_size=10, slop=sync_slop_sec
        )
        self.ts.registerCallback(self.sync_callback)

        # Reliable QoS to match BYU-FROST-Lab/seatrac-ros2
        self.publisher = self.create_publisher(
            ModemStatus, output_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def sync_callback(self, imu_msg: Imu, depth_msg: Odometry) -> None:
        """
        Combine synchronized IMU and depth data into a ModemStatus and publish it.

        :param imu_msg: Imu message containing the fused orientation.
        :param depth_msg: Odometry message containing depth data.
        """
        self.publisher.publish(self.create_modem_status_msg(imu_msg, depth_msg))

    def create_modem_status_msg(self, imu_msg: Imu, depth_msg: Odometry) -> ModemStatus:
        """
        Create a ModemStatus message with NED attitude in decidegrees and depth in decimeters.

        :param imu_msg: Imu message containing the fused orientation.
        :param depth_msg: Odometry message containing depth data.
        :return: Populated ModemStatus message, stamped with ms since node start.
        """
        modem_status_msg = ModemStatus()
        modem_status_msg.header = imu_msg.header

        modem_status_msg.msg_id = seatrac.CID_STATUS
        elapsed_ns = (self.get_clock().now() - self.start_time).nanoseconds
        modem_status_msg.timestamp = elapsed_ns // 1_000_000  # ms since start

        # Convert ENU -> NED and FLU -> FRD
        q = imu_msg.orientation
        enu_R_base = Rotation.from_quat([q.x, q.y, q.z, q.w])
        ned_R_beacon = _NED_R_ENU * enu_R_base * _FLU_R_FRD
        roll_ned, pitch_ned, yaw_ned = ned_R_beacon.as_euler("xyz", degrees=True)

        modem_status_msg.includes_local_attitude = True
        modem_status_msg.attitude_yaw = seatrac.clamp_int16(yaw_ned * 10.0)
        modem_status_msg.attitude_pitch = seatrac.clamp_int16(pitch_ned * 10.0)
        modem_status_msg.attitude_roll = seatrac.clamp_int16(roll_ned * 10.0)

        # Convert ENU -> NED
        modem_status_msg.includes_env_fields = True
        modem_status_msg.depth_local = seatrac.clamp_int16(
            -depth_msg.pose.pose.position.z * 10.0
        )

        return modem_status_msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    modem_status_converter_node = ModemStatusConverterNode()
    try:
        rclpy.spin(modem_status_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        modem_status_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
