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

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image


class DepthCameraConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_camera_converter_node")

        self.declare_parameter("sync_slop_sec", 0.05)
        self.declare_parameter("min_range", 0.2)
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("depth_input_topic", "DepthCameraDepth")
        self.declare_parameter("info_input_topic", "DepthCameraInfo")
        self.declare_parameter("color_input_topic", "RGBCameraFront")
        self.declare_parameter("depth_output_topic", "depth/image_rect")
        self.declare_parameter("info_output_topic", "depth/camera_info")
        self.declare_parameter("color_output_topic", "depth/image_rect_color")
        self.declare_parameter("depth_camera_frame", "depth_camera_link")

        self._min_range = self.get_parameter("min_range").value
        self._max_range = self.get_parameter("max_range").value
        self._depth_camera_frame = self.get_parameter("depth_camera_frame").value

        self._depth_pub = self.create_publisher(
            Image,
            self.get_parameter("depth_output_topic").value,
            qos_profile_sensor_data,
        )
        self._info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("info_output_topic").value,
            qos_profile_sensor_data,
        )
        self._color_pub = self.create_publisher(
            Image,
            self.get_parameter("color_output_topic").value,
            qos_profile_sensor_data,
        )

        self._depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter("depth_input_topic").value,
            qos_profile=qos_profile_system_default,
        )
        self._info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            self.get_parameter("info_input_topic").value,
            qos_profile=qos_profile_system_default,
        )
        self._color_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter("color_input_topic").value,
            qos_profile=qos_profile_system_default,
        )

        self._time_sync = message_filters.ApproximateTimeSynchronizer(
            [self._depth_sub, self._info_sub, self._color_sub],
            queue_size=10,
            slop=self.get_parameter("sync_slop_sec").value,
        )
        self._time_sync.registerCallback(self._sync_callback)

        self.get_logger().info("Initialization complete.")

    def sync_callback(
        self, depth_msg: Image, info_msg: CameraInfo, color_msg: Image
    ) -> None:
        for msg in (depth_msg, info_msg, color_msg):
            msg.header.frame_id = self._depth_camera_frame
        for msg in (info_msg, color_msg):
            msg.header.stamp = depth_msg.header.stamp

        depth = (
            np.frombuffer(depth_msg.data, np.float32)
            .reshape(depth_msg.height, depth_msg.width)
            .copy()
        )
        depth[(depth < self._min_range) | (depth > self._max_range)] = np.nan
        depth_msg.data = depth.tobytes()

        self._depth_pub.publish(depth_msg)
        self._info_pub.publish(info_msg)
        self._color_pub.publish(color_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    depth_camera_converter_node = DepthCameraConverterNode()
    try:
        rclpy.spin(depth_camera_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_camera_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
