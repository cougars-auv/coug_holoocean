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
import random

import rclpy
from dvl_msgs.msg import DVLDR, ConfigCommand
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener

_NED_R_ENU = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]).inv()
_FLU_R_FRD = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


class DvlOdomConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("dvl_odom_converter_node")

        self.declare_parameter("noise_sigma_scale", 0.0101)
        self.declare_parameter("yaw_drift_sigma", 0.3)
        self.declare_parameter("add_noise", True)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("input_topic", "DynamicsSensorOdom")
        self.declare_parameter("output_topic", "dvl/position")
        self.declare_parameter("config_command_topic", "dvl/config/command")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dvl_frame", "dvl_link")
        self.declare_parameter("map_frame", "map")

        self._noise_sigma_scale = self.get_parameter("noise_sigma_scale").value
        self._yaw_drift_sigma = self.get_parameter("yaw_drift_sigma").value
        self._add_noise = self.get_parameter("add_noise").value
        self._tf_timeout_sec = self.get_parameter("tf_timeout_sec").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        config_command_topic = self.get_parameter("config_command_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._dvl_frame = self.get_parameter("dvl_frame").value
        self._map_frame = self.get_parameter("map_frame").value

        self._ref_position = (0.0, 0.0, 0.0)
        self._ref_rotation = Rotation.identity()
        self._ref_stamp = None
        self._last_position = None
        self._dr_position = (0.0, 0.0, 0.0)
        self._reset_pending = False
        self._reset_drift()

        self._output_pub = self.create_publisher(
            DVLDR, output_topic, qos_profile_sensor_data
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._input_sub = self.create_subscription(
            Odometry, input_topic, self._odom_callback, qos_profile_system_default
        )
        self._config_sub = self.create_subscription(
            ConfigCommand,
            config_command_topic,
            self._config_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Initialization complete.")

    def _reset_drift(self) -> None:
        self._distance_traveled = 0.0
        self._scale_error = 0.0
        self._yaw_drift_rate = 0.0

        if self._add_noise:
            self._scale_error = random.gauss(0, self._noise_sigma_scale)
            self._yaw_drift_rate = random.gauss(0, self._yaw_drift_sigma)

    def _config_callback(self, msg: ConfigCommand) -> None:
        if msg.command != "reset_dead_reckoning":
            return

        self._reset_pending = True
        self.get_logger().info("DVL dead reckoning reset.")

    def _odom_callback(self, msg: Odometry) -> None:
        holo_T_base = PoseStamped()
        holo_T_base.header = msg.header
        holo_T_base.pose = msg.pose.pose

        try:
            map_T_base = self._tf_buffer.transform(
                holo_T_base,
                self._map_frame,
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout_sec),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Could not transform {msg.header.frame_id} to {self._map_frame}: {e}",
                throttle_duration_sec=1.0,
            )
            return

        try:
            base_T_dvl_tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._dvl_frame, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Could not transform {self._base_frame} to {self._dvl_frame}: {e}",
                throttle_duration_sec=1.0,
            )
            return

        # Transform the base pose to the DVL pose, both in the map frame
        map_T_base_tf = TransformStamped()
        map_T_base_tf.header = map_T_base.header
        map_T_base_tf.child_frame_id = self._base_frame
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
        ned_x = map_T_dvl.position.y
        ned_y = map_T_dvl.position.x
        ned_z = -map_T_dvl.position.z

        q = map_T_dvl.orientation
        enu_R_dvl = Rotation.from_quat([q.x, q.y, q.z, q.w])
        ned_R_dvl = _NED_R_ENU * enu_R_dvl

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ned_position = (ned_x, ned_y, ned_z)

        if self._reset_pending:
            self._ref_position = ned_position
            self._ref_rotation = ned_R_dvl
            self._ref_stamp = stamp
            self._last_position = None
            self._reset_drift()
            self._reset_pending = False

        if self._ref_stamp is None:
            self._ref_stamp = stamp

        ref_R_ned = self._ref_rotation.inv()
        elapsed_minutes = (stamp - self._ref_stamp) / 60.0
        yaw_error = Rotation.from_euler(
            "z", self._yaw_drift_rate * elapsed_minutes, degrees=True
        )

        if self._last_position is None:
            self._dr_position = ref_R_ned.apply(
                [
                    pos - ref
                    for pos, ref in zip(ned_position, self._ref_position, strict=True)
                ]
            )
        else:
            delta_position = ref_R_ned.apply(
                [
                    pos - last
                    for pos, last in zip(ned_position, self._last_position, strict=True)
                ]
            )
            self._distance_traveled += math.dist(ned_position, self._last_position)
            self._dr_position += yaw_error.apply(delta_position) * (
                1.0 + self._scale_error
            )

        self._last_position = ned_position

        ned_x, ned_y, ned_z = self._dr_position
        ned_R_dvl = yaw_error * ref_R_ned * ned_R_dvl

        # Convert FLU -> FRD
        ned_roll, ned_pitch, ned_yaw = (ned_R_dvl * _FLU_R_FRD).as_euler(
            "xyz", degrees=True
        )

        dvl_msg = DVLDR()
        dvl_msg.header.stamp = msg.header.stamp
        dvl_msg.header.frame_id = self._dvl_frame
        dvl_msg.time = stamp
        dvl_msg.position.x = ned_x
        dvl_msg.position.y = ned_y
        dvl_msg.position.z = ned_z
        dvl_msg.pos_std = self._noise_sigma_scale * self._distance_traveled
        dvl_msg.roll = ned_roll
        dvl_msg.pitch = ned_pitch
        dvl_msg.yaw = ned_yaw
        dvl_msg.status = 0

        self._output_pub.publish(dvl_msg)


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
