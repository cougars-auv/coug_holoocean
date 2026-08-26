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


from coug_holoocean.utils import seatrac_enums as seatrac


def test_amsgtype_table() -> None:
    assert seatrac.AMSGTYPE_TO_MSG_TYPE[0] == "OWAY"
    assert seatrac.AMSGTYPE_TO_MSG_TYPE[7] == "MSG_RESPX"
    assert sorted(seatrac.AMSGTYPE_TO_MSG_TYPE) == list(range(8))


def test_req_resp_consistency() -> None:
    assert seatrac.RESP_TYPES == set(seatrac.REQ_TO_RESP.values())
    assert set(seatrac.REQ_TO_RESP).isdisjoint(seatrac.RESP_TYPES)


def test_field_flag_sets_reference_known_types() -> None:
    known = set(seatrac.AMSGTYPE_TO_MSG_TYPE.values())
    for flags in (seatrac.HAS_USBL, seatrac.HAS_RANGE, seatrac.HAS_Z):
        assert flags <= known


def test_clamp_int16() -> None:
    assert seatrac.clamp_int16(0.0) == 0
    assert seatrac.clamp_int16(40000) == 32767
    assert seatrac.clamp_int16(-40000) == -32768
    assert seatrac.clamp_int16(1.5) == 2
    assert seatrac.clamp_int16(-1.5) == -2


def test_clamp_uint16() -> None:
    assert seatrac.clamp_uint16(0.0) == 0
    assert seatrac.clamp_uint16(-5) == 0
    assert seatrac.clamp_uint16(100000) == 65535
    assert seatrac.clamp_uint16(2.5) == 2
