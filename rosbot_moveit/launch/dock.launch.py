# Copyright 2024 Husarion sp. z o.o.
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
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    moveit_config = (
        MoveItConfigsBuilder("rosbot_xl", package_name="rosbot_moveit").joint_limits(
            file_path="config/joint_limits.yaml"
        )
    ).to_moveit_configs()

    # Same SRDF override as move_group.launch.py, and needed for the same
    # reason: the gripper's Close named state depends on which claw is
    # rendered (-0.023 with the simulation-only CAD claw, -0.009 on hardware,
    # where -0.023 is out of bounds and setJointValueTarget would reject it).
    # This node resolves setNamedTarget("Close") against its OWN semantic copy,
    # so fixing move_group alone left it closing to -0.009 and leaking the
    # 26 mm cork. ParameterValue(value_type=str) is load-bearing: launch_ros
    # YAML-parses a substitution result to infer the type, and the rendered
    # SRDF's "<!--GROUPS: ..." reads as a mapping key and raises ScannerError.
    moveit_config.robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("rosbot_moveit"), "config", "rosbot_xl.srdf"]
                    ),
                    " taper_jaws:=",
                    LaunchConfiguration("use_sim"),
                ]
            ),
            value_type=str,
        )
    }


    # Standalone-only (no auto-dock equivalent of home). Uses Node(namespace=)
    # not PushRosNamespace so empty default won't double-stack a parent ns.
    dock_node = Node(
        package="rosbot_moveit",
        executable="dock",
        namespace=namespace,
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim",
                default_value="False",
                description=(
                    "Simulation mode. Also selects the gripper's Close named "
                    "state, which differs because the simulation-only claw "
                    "closes further than the hardware jaws."
                ),
                choices=["True", "true", "False", "false"],
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace; leave empty when the robot was launched without one.",
            ),
            dock_node,
        ]
    )
