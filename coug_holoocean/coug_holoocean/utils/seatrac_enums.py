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

from enum import IntEnum, StrEnum


class AcousticMessageType(StrEnum):
    ONE_WAY = "OWAY"
    ONE_WAY_USBL = "OWAYU"
    REQUEST = "MSG_REQ"
    RESPONSE = "MSG_RESP"
    REQUEST_USBL = "MSG_REQU"
    RESPONSE_USBL = "MSG_RESPU"
    REQUEST_EXTENDED = "MSG_REQX"
    RESPONSE_EXTENDED = "MSG_RESPX"


class CommandId(IntEnum):
    STATUS = 0x10
    DAT_SEND = 0x60
    DAT_RECEIVE = 0x61
    DAT_ERROR = 0x63
    DAT_QUEUE_SET = 0x64


class CommandStatus(IntEnum):
    OK = 0x00
    TRANSCEIVER_BUSY = 0x30
    TRANSCEIVER_RESPONSE_TIMEOUT = 0x34


AMSGTYPE_TO_MSG_TYPE: dict[int, AcousticMessageType] = {
    0: AcousticMessageType.ONE_WAY,
    1: AcousticMessageType.ONE_WAY_USBL,
    2: AcousticMessageType.REQUEST,
    3: AcousticMessageType.RESPONSE,
    4: AcousticMessageType.REQUEST_USBL,
    5: AcousticMessageType.RESPONSE_USBL,
    6: AcousticMessageType.REQUEST_EXTENDED,
    7: AcousticMessageType.RESPONSE_EXTENDED,
}

REQ_TO_RESP: dict[AcousticMessageType, AcousticMessageType] = {
    AcousticMessageType.REQUEST: AcousticMessageType.RESPONSE,
    AcousticMessageType.REQUEST_USBL: AcousticMessageType.RESPONSE_USBL,
    AcousticMessageType.REQUEST_EXTENDED: AcousticMessageType.RESPONSE_EXTENDED,
}
RESP_TYPES = frozenset(REQ_TO_RESP.values())

HAS_USBL = frozenset(
    {
        AcousticMessageType.ONE_WAY_USBL,
        AcousticMessageType.REQUEST_USBL,
        AcousticMessageType.RESPONSE_USBL,
        AcousticMessageType.REQUEST_EXTENDED,
        AcousticMessageType.RESPONSE_EXTENDED,
    }
)
HAS_RANGE = frozenset(
    {
        AcousticMessageType.RESPONSE,
        AcousticMessageType.RESPONSE_USBL,
        AcousticMessageType.RESPONSE_EXTENDED,
    }
)
HAS_Z = frozenset(
    {
        AcousticMessageType.RESPONSE_USBL,
        AcousticMessageType.RESPONSE_EXTENDED,
    }
)

METERS_TO_DECIMETERS = 10.0
DEGREES_TO_DECIDEGREES = 10.0


def clamp_int16(v: float) -> int:
    return max(-32768, min(32767, round(v)))


def clamp_uint16(v: float) -> int:
    return max(0, min(65535, round(v)))
