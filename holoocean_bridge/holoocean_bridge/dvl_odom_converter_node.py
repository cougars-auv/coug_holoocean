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
from dvl_msgs.msg import DVLDR
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

_NED_R_ENU = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]).inv()


class DvlOdomConverterNode(Node):
    """
    ROS 2 node that converts HoloOcean Odometry messages to DVLDR messages.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("dvl_odom_converter_node")

        self.declare_parameter("pos_std", 0.05)
        self.declare_parameter("input_topic", "DynamicsSensorOdom")
        self.declare_parameter("output_topic", "dvl/position")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("map_frame", "map")
        self.pos_std = self.get_parameter("pos_std").get_parameter_value().double_value
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.dvl_frame = (
            self.get_parameter("dvl_frame").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(
            DVLDR, output_topic, qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def listener_callback(self, msg: Odometry) -> None:
        """
        Transform HoloOcean ground truth odometry into the DVL frame and publish.

        :param msg: Odometry message containing the base pose in the HoloOcean frame.
        """
        holo_T_base = PoseStamped()
        holo_T_base.header = msg.header
        holo_T_base.pose = msg.pose.pose

        try:
            map_T_base = self.tf_buffer.transform(
                holo_T_base,
                self.map_frame,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f"Could not transform {msg.header.frame_id} to {self.map_frame}: {e}",
                throttle_duration_sec=1.0,
            )
            return

        try:
            base_T_dvl_tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.dvl_frame, rclpy.time.Time()
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f"Could not transform {self.base_frame} to {self.dvl_frame}: {e}",
                throttle_duration_sec=1.0,
            )
            return

        # Transform from base-frame to DVL-frame pose in the map frame
        map_T_base_tf = TransformStamped()
        map_T_base_tf.header = map_T_base.header
        map_T_base_tf.child_frame_id = self.base_frame
        map_T_base_tf.transform.translation.x = map_T_base.pose.position.x
        map_T_base_tf.transform.translation.y = map_T_base.pose.position.y
        map_T_base_tf.transform.translation.z = map_T_base.pose.position.z
        map_T_base_tf.transform.rotation = map_T_base.pose.orientation

        base_T_dvl = PoseStamped()
        base_T_dvl.pose.position.x = base_T_dvl_tf.transform.translation.x
        base_T_dvl.pose.position.y = base_T_dvl_tf.transform.translation.y
        base_T_dvl.pose.position.z = base_T_dvl_tf.transform.translation.z
        base_T_dvl.pose.orientation = base_T_dvl_tf.transform.rotation

        map_T_dvl = do_transform_pose(base_T_dvl.pose, map_T_base_tf)

        # Convert ENU -> NED
        x_ned = map_T_dvl.position.y
        y_ned = map_T_dvl.position.x
        z_ned = -map_T_dvl.position.z

        q = map_T_dvl.orientation
        enu_R_dvl = Rotation.from_quat([q.x, q.y, q.z, q.w])
        ned_R_dvl = _NED_R_ENU * enu_R_dvl
        roll_ned, pitch_ned, yaw_ned = ned_R_dvl.as_euler("xyz", degrees=True)

        dvl_msg = DVLDR()
        dvl_msg.header.stamp = msg.header.stamp
        dvl_msg.header.frame_id = self.dvl_frame
        dvl_msg.time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dvl_msg.position.x = x_ned
        dvl_msg.position.y = y_ned
        dvl_msg.position.z = z_ned
        dvl_msg.pos_std = self.pos_std
        dvl_msg.roll = roll_ned
        dvl_msg.pitch = pitch_ned
        dvl_msg.yaw = yaw_ned
        dvl_msg.status = 0

        self.publisher.publish(dvl_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    dvl_odom_converter_node = DvlOdomConverterNode()
    try:
        rclpy.spin(dvl_odom_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_odom_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
