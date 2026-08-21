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
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        "rosbot_xl", package_name="rosbot_moveit"
    ).to_moveit_configs()

    components_config = PathJoinSubstitution(
        [FindPackageShare("rosbot_description"), "config", "rosbot_xl", "manipulation.yaml"]
    )

    # Override builder's vanilla xacro so URDF matches bringup
    # (components_config + configuration:='manipulation').
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "urdf",
                    "rosbot_xl.urdf.xacro",
                ]
            ),
            " components_config:=",
            components_config,
            " configuration:='manipulation'",
        ]
    )
    moveit_config.robot_description = {"robot_description": robot_description_content}

    # Override the builder's SRDF the same way, and for the same reason: the
    # gripper's Close named state depends on which claw is rendered. body.xacro
    # sets the lower stop to -0.023 with the simulation-only CAD claw and
    # -0.010 otherwise, so a single value cannot serve both. Without this the
    # sim closes to -0.009 and leaves a 28 mm gap the 26 mm cork slips through.
    # MoveItConfigsBuilder already runs the SRDF through load_xacro(), so the
    # file needs no rename to accept the argument.
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rosbot_moveit"), "config", "rosbot_xl.srdf"]
            ),
            " taper_jaws:=",
            LaunchConfiguration("use_sim"),
        ]
    )
    # value_type=str is load-bearing, not boilerplate. launch_ros YAML-parses a
    # substitution's result to infer the parameter type, and the rendered SRDF
    # starts its comment block with "<!--GROUPS: Representation of..." -- YAML
    # reads that "GROUPS:" as a mapping key and raises ScannerError, which
    # aborts the whole launch before Gazebo comes up. The URDF override above
    # gets away with a bare Command only because its render happens to contain
    # no "key: value" pattern; that is luck, not a working pattern to copy.
    moveit_config.robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        # OccupancyMapMonitor runs unconditionally inside PlanningSceneMonitor;
        # without sensors_3d.yaml it logs one ERROR + WARN at startup. Accepted.
        "disable_capabilities": "",
        "monitor_dynamics": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description=(
            "Simulation mode. Also selects the gripper's Close named state, "
            "which differs because the simulation-only claw closes further."
        ),
        choices=["True", "true", "False", "false"],
    )

    return LaunchDescription([declare_use_sim_arg, move_group_node])
