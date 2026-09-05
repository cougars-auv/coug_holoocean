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

from coug_holoocean.utils import seatrac_enums as seatrac


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
