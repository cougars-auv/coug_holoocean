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
    def __init__(self) -> None:
        super().__init__("truth_converter_node")

        self.declare_parameter("publish_tf", False)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("input_topic", "DynamicsSensorOdom")
        self.declare_parameter("output_topic", "odometry/truth")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.publish_tf = self.get_parameter("publish_tf").value
        self.tf_timeout_sec = self.get_parameter("tf_timeout_sec").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        self.output_pub = self.create_publisher(
            Odometry, output_topic, qos_profile_system_default
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.input_sub = self.create_subscription(
            Odometry, input_topic, self.odom_callback, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def odom_callback(self, msg: Odometry) -> None:
        holo_T_base = PoseStamped()
        holo_T_base.header = msg.header
        holo_T_base.pose = msg.pose.pose

        try:
            map_T_base = self.tf_buffer.transform(
                holo_T_base,
                self.map_frame,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
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

        self.output_pub.publish(odom_msg)

        if self.publish_tf:
            map_T_base_tf = TransformStamped()
            map_T_base_tf.header.stamp = msg.header.stamp
            map_T_base_tf.header.frame_id = self.map_frame
            map_T_base_tf.child_frame_id = self.base_frame
            map_T_base_tf.transform.translation.x = map_T_base.pose.position.x
            map_T_base_tf.transform.translation.y = map_T_base.pose.position.y
            map_T_base_tf.transform.translation.z = map_T_base.pose.position.z
            map_T_base_tf.transform.rotation = map_T_base.pose.orientation
            self.tf_broadcaster.sendTransform(map_T_base_tf)


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
