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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def agent_frame(agent_ns: LaunchConfiguration, frame: str) -> PythonExpression:
    return PythonExpression(
        ["'", agent_ns, f"/{frame}' if '", agent_ns, f"' != '' else '{frame}'"]
    )


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")
    add_noise = LaunchConfiguration("add_noise")

    fleet_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            "fleet",
            "coug_holoocean_params.yaml",
        ]
    )
    agent_param_file = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_DIR"),
            PythonExpression(["'", agent_ns, "' + '_params.yaml'"]),
        ]
    )

    depth_camera_link_frame = agent_frame(agent_ns, "depth_camera_link")
    depth_link_frame = agent_frame(agent_ns, "depth_link")
    dvl_link_frame = agent_frame(agent_ns, "dvl_link")
    base_link_frame = agent_frame(agent_ns, "base_link")
    gps_link_frame = agent_frame(agent_ns, "gps_link")
    imu_link_frame = agent_frame(agent_ns, "imu_link")
    modem_link_frame = agent_frame(agent_ns, "modem_link")
    front_stereo_link_frame = agent_frame(agent_ns, "front_stereo_link")
    back_stereo_link_frame = agent_frame(agent_ns, "back_stereo_link")
    com_link_frame = agent_frame(agent_ns, "com_link")

    agent_name = PythonExpression(
        ["'", agent_ns, "' if '", agent_ns, "' != '' else 'auv0'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="auv0",
                description="Namespace for the agent (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "add_noise",
                default_value="true",
                description="Whether to add noise to sensor data",
            ),
            Node(
                package="coug_holoocean",
                executable="cmd_vel_converter",
                name="cmd_vel_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {"use_sim_time": use_sim_time, "agent_name": agent_name},
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="depth_camera_converter",
                name="depth_camera_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "depth_camera_frame": depth_camera_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="depth_converter",
                name="depth_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "depth_frame": depth_link_frame,
                        "map_frame": "map",
                        "add_noise": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="dvl_converter",
                name="dvl_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "dvl_frame": dvl_link_frame,
                        "add_noise": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="dvl_odom_converter",
                name="dvl_odom_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "base_frame": base_link_frame,
                        "dvl_frame": dvl_link_frame,
                        "map_frame": "map",
                        "add_noise": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="fin_state_publisher",
                name="fin_state_publisher_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="gps_converter",
                name="gps_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "gps_frame": gps_link_frame,
                        "add_noise": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="hsd_converter",
                name="hsd_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {"use_sim_time": use_sim_time, "agent_name": agent_name},
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="imu_converter",
                name="imu_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "imu_frame": imu_link_frame,
                        "add_noise": add_noise,
                        "add_bias": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="mag_converter",
                name="mag_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "mag_frame": imu_link_frame,
                        "add_noise": add_noise,
                        "add_bias": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="modem_converter",
                name="modem_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "modem_frame": modem_link_frame,
                        "add_noise": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="depth_converter",
                name="modem_depth_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "depth_frame": modem_link_frame,
                        "map_frame": "map",
                        "add_noise": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="imu_converter",
                name="modem_imu_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "imu_frame": modem_link_frame,
                        "add_noise": add_noise,
                        "add_bias": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="modem_status_converter",
                name="modem_status_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="pressure_converter",
                name="pressure_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "depth_frame": depth_link_frame,
                        "add_noise": add_noise,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="stereo_converter",
                name="stereo_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "front_stereo_frame": front_stereo_link_frame,
                        "back_stereo_frame": back_stereo_link_frame,
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="truth_converter",
                name="truth_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "base_frame": base_link_frame,
                        "map_frame": "map",
                    },
                ],
            ),
            Node(
                package="coug_holoocean",
                executable="wrench_converter",
                name="wrench_converter_node",
                parameters=[
                    fleet_param_file,
                    agent_param_file,
                    {
                        "use_sim_time": use_sim_time,
                        "wrench_frame": com_link_frame,
                    },
                ],
            ),
        ]
    )
