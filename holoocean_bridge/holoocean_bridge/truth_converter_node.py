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

import rclpy
import tf2_geometry_msgs  # noqa: F401
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class TruthConverterNode(Node):
    """
    ROS 2 node that converts HoloOcean ground truth Odometry messages to Odometry messages.

    Optionally publishes the map->base_link transform.

    :author: Nelson Durrant
    :date: May 2026
    """

    def __init__(self) -> None:
        super().__init__("truth_converter_node")

        self.declare_parameter("publish_tf", False)
        self.declare_parameter("input_topic", "DynamicsSensorOdom")
        self.declare_parameter("output_topic", "odometry/truth")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.publish_tf = (
            self.get_parameter("publish_tf").get_parameter_value().bool_value
        )
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(
            Odometry, output_topic, qos_profile_system_default
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def listener_callback(self, msg: Odometry) -> None:
        """
        Transform HoloOcean ground truth odometry into the map frame and publish.

        :param msg: Odometry message from DynamicsSensorOdom (base in HoloOcean frame).
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

        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = map_T_base.pose
        odom_msg.pose.covariance = msg.pose.covariance
        odom_msg.twist.covariance = msg.twist.covariance

        self.publisher.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = map_T_base.pose.position.x
            t.transform.translation.y = map_T_base.pose.position.y
            t.transform.translation.z = map_T_base.pose.position.z
            t.transform.rotation = map_T_base.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    truth_converter_node = TruthConverterNode()
    try:
        rclpy.spin(truth_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        truth_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
