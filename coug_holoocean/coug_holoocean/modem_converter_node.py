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
from holoocean_interfaces.msg import AcousticBeaconSend, AcousticBeaconSensor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from seatrac_interfaces.msg import ModemCmdUpdate, ModemRec, ModemSend

from coug_holoocean.utils import seatrac_enums as seatrac


class ModemConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("modem_converter_node")

        self.declare_parameter("tick_period_sec", 0.1)
        self.declare_parameter("send_delay_sec", 0.4)
        self.declare_parameter("resp_delay_sec", 0.0)
        self.declare_parameter("resp_timeout_sec", 4.0)
        self.declare_parameter("beacon_id", 1)
        self.declare_parameter("bearing_noise_sigmas", [0.01745, 0.01745])
        self.declare_parameter("range_noise_sigma", 0.1)
        self.declare_parameter("add_noise", True)
        self.declare_parameter("beacon_rec_topic", "AcousticBeaconSensor")
        self.declare_parameter("beacon_send_topic", "/acoustic_beacon_send")
        self.declare_parameter("modem_rec_topic", "modem_rec")
        self.declare_parameter("modem_send_topic", "modem_send")
        self.declare_parameter("modem_cmd_update_topic", "modem_cmd_update")
        self.declare_parameter("depth_topic", "modem/depth/odometry")
        self.declare_parameter("modem_frame", "modem_link")

        self._tick_period_sec = self.get_parameter("tick_period_sec").value
        self._send_delay_sec = self.get_parameter("send_delay_sec").value
        self._resp_delay_sec = self.get_parameter("resp_delay_sec").value
        self._resp_timeout_sec = self.get_parameter("resp_timeout_sec").value
        self._beacon_id = self.get_parameter("beacon_id").value
        self._bearing_noise_sigmas = self.get_parameter("bearing_noise_sigmas").value
        self._range_noise_sigma = self.get_parameter("range_noise_sigma").value
        self._add_noise = self.get_parameter("add_noise").value
        beacon_rec_topic = self.get_parameter("beacon_rec_topic").value
        beacon_send_topic = self.get_parameter("beacon_send_topic").value
        modem_rec_topic = self.get_parameter("modem_rec_topic").value
        modem_send_topic = self.get_parameter("modem_send_topic").value
        modem_cmd_update_topic = self.get_parameter("modem_cmd_update_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        self._modem_frame = self.get_parameter("modem_frame").value

        self._send_delay_ticks = max(
            1, round(self._send_delay_sec / self._tick_period_sec)
        )
        self._resp_delay_ticks = max(
            0, round(self._resp_delay_sec / self._tick_period_sec)
        )
        self._resp_timeout_ticks = max(
            1, round(self._resp_timeout_sec / self._tick_period_sec)
        )

        self._send_queue = []
        self._pending_auto_responses = []
        self._pending_resp_target = None
        self._send_delay_ticker = 0
        self._pending_resp_ticker = 0
        self._dat_queue = {}

        self._agent_depth = 0.0

        self._beacon_rec_sub = self.create_subscription(
            AcousticBeaconSensor,
            beacon_rec_topic,
            self._beacon_callback,
            qos_profile_system_default,
        )
        self._modem_send_sub = self.create_subscription(
            ModemSend,
            modem_send_topic,
            self._modem_send_callback,
            qos_profile_system_default,
        )
        self._depth_sub = self.create_subscription(
            Odometry,
            depth_topic,
            self._depth_callback,
            qos_profile_system_default,
        )

        self._modem_rec_pub = self.create_publisher(
            ModemRec, modem_rec_topic, qos_profile_system_default
        )
        self._beacon_send_pub = self.create_publisher(
            AcousticBeaconSend, beacon_send_topic, qos_profile_system_default
        )
        self._modem_cmd_update_pub = self.create_publisher(
            ModemCmdUpdate, modem_cmd_update_topic, qos_profile_system_default
        )

        self._tick_timer = self.create_timer(self._tick_period_sec, self._tick_callback)

        self.get_logger().info("Initialization complete.")

    def _depth_callback(self, msg: Odometry) -> None:
        self._agent_depth = -msg.pose.pose.position.z

    def _beacon_callback(self, msg: AcousticBeaconSensor) -> None:
        self._publish_modem_rec(msg)

        # The real beacon firmware answers REQ messages with the queued data
        if msg.msg_type in seatrac.REQ_TO_RESP and msg.to_beacon == self._beacon_id:
            self._queue_auto_response(msg)

        # A RESP from the queried beacon frees the channel
        if (
            msg.msg_type in seatrac.RESP_TYPES
            and msg.from_beacon == self._pending_resp_target
        ):
            self._pending_resp_target = None
            self._pending_resp_ticker = 0
            self._attempt_send()

    def _publish_modem_rec(self, msg: AcousticBeaconSensor) -> None:
        modem_rec = ModemRec()
        modem_rec.header.stamp = msg.header.stamp
        modem_rec.header.frame_id = self._modem_frame
        modem_rec.msg_id = seatrac.CommandId.DAT_RECEIVE

        modem_rec.local_flag = msg.to_beacon in (self._beacon_id, 0)
        modem_rec.dest_id = msg.to_beacon & 0xFF
        modem_rec.src_id = msg.from_beacon & 0xFF

        modem_rec.depth_local = seatrac.clamp_int16(
            self._agent_depth * seatrac.METERS_TO_DECIMETERS
        )

        modem_rec.includes_usbl = msg.msg_type in seatrac.HAS_USBL
        if modem_rec.includes_usbl:
            # Convert FLU -> FRD
            azimuth = -msg.azimuth
            elevation = msg.elevation
            if self._add_noise:
                azimuth += random.gauss(0, self._bearing_noise_sigmas[0])
                elevation += random.gauss(0, self._bearing_noise_sigmas[1])
            modem_rec.usbl_azimuth = seatrac.clamp_int16(
                math.degrees(azimuth) * seatrac.DEGREES_TO_DECIDEGREES
            )
            modem_rec.usbl_elevation = seatrac.clamp_int16(
                math.degrees(elevation) * seatrac.DEGREES_TO_DECIDEGREES
            )
            modem_rec.usbl_channels = 4

        modem_rec.includes_range = msg.msg_type in seatrac.HAS_RANGE
        if modem_rec.includes_range:
            range_dist = msg.range
            if self._add_noise:
                range_dist += random.gauss(0, self._range_noise_sigma)
            modem_rec.range_dist = seatrac.clamp_uint16(
                range_dist * seatrac.METERS_TO_DECIMETERS
            )

        modem_rec.includes_position = msg.msg_type in seatrac.HAS_Z
        if modem_rec.includes_position:
            # TODO: Fix RESPX remote depth reading in HoloOcean (not populated)
            modem_rec.position_enhanced = False
            remote_depth = self._agent_depth - msg.range * math.sin(msg.elevation)
            modem_rec.position_depth = seatrac.clamp_int16(
                remote_depth * seatrac.METERS_TO_DECIMETERS
            )

        payload = list(msg.msg_data[:30])
        modem_rec.packet_len = len(payload)
        modem_rec.packet_data = payload + [0] * (30 - len(payload))

        self._modem_rec_pub.publish(modem_rec)

    def _queue_auto_response(self, msg: AcousticBeaconSensor) -> None:
        # Consume any payload staged for the requester (or for all beacons)
        queued = self._dat_queue.pop(int(msg.from_beacon), None) or self._dat_queue.pop(
            0, None
        )

        resp = AcousticBeaconSend()
        resp.header.stamp = self.get_clock().now().to_msg()
        resp.header.frame_id = self._modem_frame
        resp.from_beacon = self._beacon_id
        resp.to_beacon = int(msg.from_beacon)
        resp.msg_type = seatrac.REQ_TO_RESP[msg.msg_type]
        resp.msg_data = queued or []

        if self._resp_delay_ticks <= 0:
            self._send_queue.append((resp, False))
        else:
            self._pending_auto_responses.append([resp, self._resp_delay_ticks])
        self._attempt_send()

    def _modem_send_callback(self, msg: ModemSend) -> None:
        if msg.msg_id == seatrac.CommandId.DAT_QUEUE_SET:
            self._set_dat_queue(msg)
            return

        if msg.msg_id != seatrac.CommandId.DAT_SEND:
            self.get_logger().warning(
                f"Unsupported send CID 0x{msg.msg_id:02X}. Dropping message."
            )
            return

        beacon_send = AcousticBeaconSend()
        beacon_send.header = msg.header
        beacon_send.from_beacon = self._beacon_id
        beacon_send.to_beacon = int(msg.dest_id)
        beacon_send.msg_type = seatrac.AMSGTYPE_TO_MSG_TYPE.get(
            msg.msg_type, seatrac.AcousticMessageType.ONE_WAY
        )
        beacon_send.msg_data = list(msg.packet_data[: msg.packet_len])

        self._send_queue.append((beacon_send, True))
        self._attempt_send()

    def _set_dat_queue(self, msg: ModemSend) -> None:
        payload = list(msg.packet_data[: msg.packet_len])
        if payload:
            self._dat_queue[int(msg.dest_id)] = payload
        else:
            self._dat_queue.pop(int(msg.dest_id), None)

        self._publish_cmd_update(seatrac.CommandId.DAT_QUEUE_SET, msg.dest_id)

    def _tick_callback(self) -> None:
        # A REQ that never gets a RESP eventually times out and frees the channel
        if self._pending_resp_target is not None:
            self._pending_resp_ticker += 1
            if self._pending_resp_ticker >= self._resp_timeout_ticks:
                self._publish_cmd_update(
                    seatrac.CommandId.DAT_ERROR,
                    self._pending_resp_target,
                    seatrac.CommandStatus.TRANSCEIVER_RESPONSE_TIMEOUT,
                )
                self._pending_resp_target = None
                self._pending_resp_ticker = 0

        self._release_auto_responses()
        self._attempt_send()

        if self._send_delay_ticker > 0:
            self._send_delay_ticker -= 1

    def _release_auto_responses(self) -> None:
        ready = []
        for item in self._pending_auto_responses:
            item[1] -= 1
            if item[1] <= 0:
                ready.append(item)

        for item in ready:
            self._pending_auto_responses.remove(item)
            self._send_queue.append((item[0], False))

    def _attempt_send(self) -> None:
        if not self._send_queue or self._send_delay_ticker > 0:
            return

        beacon_send, is_command = self._send_queue[0]

        # The channel is held while a prior REQ awaits its RESP
        if self._pending_resp_target is not None:
            if is_command:
                self._publish_cmd_update(
                    seatrac.CommandId.DAT_SEND,
                    beacon_send.to_beacon,
                    seatrac.CommandStatus.TRANSCEIVER_BUSY,
                )
            self._send_delay_ticker = self._send_delay_ticks
            return

        self._send_queue.pop(0)
        self._beacon_send_pub.publish(beacon_send)

        if is_command:
            # Transmission always succeeds in sim
            self._publish_cmd_update(seatrac.CommandId.DAT_SEND, beacon_send.to_beacon)

        # A REQ holds the channel until its RESP arrives (or times out)
        if beacon_send.msg_type in seatrac.REQ_TO_RESP:
            self._pending_resp_target = int(beacon_send.to_beacon)
            self._pending_resp_ticker = 0

        self._send_delay_ticker = self._send_delay_ticks

    def _publish_cmd_update(
        self,
        msg_id: seatrac.CommandId,
        target_id: int,
        status: seatrac.CommandStatus = seatrac.CommandStatus.OK,
    ) -> None:
        cmd_update = ModemCmdUpdate()
        cmd_update.header.stamp = self.get_clock().now().to_msg()
        cmd_update.msg_id = msg_id
        cmd_update.command_status_code = status
        cmd_update.target_id = target_id & 0xFF
        cmd_update.queue_size = len(self._send_queue)
        cmd_update.time_sent = cmd_update.header.stamp

        self._modem_cmd_update_pub.publish(cmd_update)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    modem_converter_node = ModemConverterNode()
    try:
        rclpy.spin(modem_converter_node)
    except KeyboardInterrupt:
        pass
    finally:
        modem_converter_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
