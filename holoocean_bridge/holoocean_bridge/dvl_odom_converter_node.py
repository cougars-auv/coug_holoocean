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
from tf2_ros import Buffer, TransformListener

_NED_R_ENU = Rotation.from_quat([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]).inv()
_FLU_R_FRD = Rotation.from_quat([1.0, 0.0, 0.0, 0.0])


class DvlOdomConverterNode(Node):
    """
    ROS 2 node that converts HoloOcean Odometry messages to noisy DVLDR messages.

    Models dead-reckoning drift as a velocity scale error and a drifting yaw estimate.
    Resets the dead-reckoning frame from a ConfigCommand like the real driver.

    :author: Nelson Durrant
    :date: May 2026
    """

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

        self.noise_sigma_scale = (
            self.get_parameter("noise_sigma_scale").get_parameter_value().double_value
        )
        self.yaw_drift_sigma = (
            self.get_parameter("yaw_drift_sigma").get_parameter_value().double_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )
        self.tf_timeout_sec = (
            self.get_parameter("tf_timeout_sec").get_parameter_value().double_value
        )
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        config_command_topic = (
            self.get_parameter("config_command_topic")
            .get_parameter_value()
            .string_value
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

        self.ref_position = (0.0, 0.0, 0.0)
        self.ref_rotation = Rotation.identity()
        self.ref_stamp = None
        self.last_position = None
        self.dr_position = (0.0, 0.0, 0.0)
        self.reset_pending = False
        self.reset_drift()

        self.publisher = self.create_publisher(
            DVLDR, output_topic, qos_profile_sensor_data
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Odometry, input_topic, self.listener_callback, qos_profile_system_default
        )
        self.config_subscription = self.create_subscription(
            ConfigCommand,
            config_command_topic,
            self.config_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Initialization complete.")

    def reset_drift(self) -> None:
        """Zero the distance traveled and redraw the dead-reckoning drift errors."""
        self.distance_traveled = 0.0
        self.scale_error = 0.0
        self.yaw_drift_rate = 0.0

        if self.add_noise:
            self.scale_error = random.gauss(0, self.noise_sigma_scale)
            self.yaw_drift_rate = random.gauss(0, self.yaw_drift_sigma)

    def config_callback(self, msg: ConfigCommand) -> None:
        """
        Re-anchor the dead-reckoning frame from a DVL reset_dead_reckoning command.

        :param msg: ConfigCommand message containing DVL config data.
        """
        if msg.command != "reset_dead_reckoning":
            return

        self.reset_pending = True
        self.get_logger().info("DVL dead reckoning reset.")

    def listener_callback(self, msg: Odometry) -> None:
        """
        Transform HoloOcean ground truth odometry into the DVL frame, add drift, and publish it.

        :param msg: Odometry message containing the base pose in the HoloOcean frame.
        """
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

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position_ned = (x_ned, y_ned, z_ned)

        if self.reset_pending:
            self.ref_position = position_ned
            self.ref_rotation = ned_R_dvl
            self.ref_stamp = stamp
            self.last_position = None
            self.reset_drift()
            self.reset_pending = False

        if self.ref_stamp is None:
            self.ref_stamp = stamp

        # The yaw estimate drifts from truth over time, dragging the position with it
        ref_R_ned = self.ref_rotation.inv()
        elapsed_min = (stamp - self.ref_stamp) / 60.0
        yaw_error = Rotation.from_euler(
            "z", self.yaw_drift_rate * elapsed_min, degrees=True
        )

        if self.last_position is None:
            self.dr_position = ref_R_ned.apply(
                [p - ref for p, ref in zip(position_ned, self.ref_position)]
            )
        else:
            step = ref_R_ned.apply(
                [p - last for p, last in zip(position_ned, self.last_position)]
            )
            self.distance_traveled += math.dist(position_ned, self.last_position)
            self.dr_position += yaw_error.apply(step) * (1.0 + self.scale_error)

        self.last_position = position_ned

        x_ned, y_ned, z_ned = self.dr_position
        ned_R_dvl = yaw_error * ref_R_ned * ned_R_dvl

        # Convert FLU -> FRD
        roll_ned, pitch_ned, yaw_ned = (ned_R_dvl * _FLU_R_FRD).as_euler(
            "xyz", degrees=True
        )

        dvl_msg = DVLDR()
        dvl_msg.header.stamp = msg.header.stamp
        dvl_msg.header.frame_id = self.dvl_frame
        dvl_msg.time = stamp
        dvl_msg.position.x = x_ned
        dvl_msg.position.y = y_ned
        dvl_msg.position.z = z_ned
        dvl_msg.pos_std = self.noise_sigma_scale * self.distance_traveled
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
