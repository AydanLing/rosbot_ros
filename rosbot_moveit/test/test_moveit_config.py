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

import os

import pytest
from conftest import load_yaml, share, srdf_root


def test_srdf_groups_and_named_states():
    """SRDF declares the groups + named states arm_pose_mover/joy2servo use."""
    root = srdf_root()

    group_names = {g.attrib["name"] for g in root.findall("group")}
    assert group_names == {"manipulator", "gripper"}, group_names

    states = {(s.attrib["name"], s.attrib["group"]) for s in root.findall("group_state")}
    required = {
        ("Home", "manipulator"),
        ("Dock", "manipulator"),
        ("Zero", "manipulator"),
        ("Open", "gripper"),
        ("Close", "gripper"),
    }
    missing = required - states
    assert not missing, f"Missing SRDF named states: {missing}"


def test_kinematics_keeps_position_only_ik():
    """4-DoF arm: KDL needs position_only_ik to satisfy IK at all."""
    cfg = load_yaml("config/kinematics.yaml")
    assert cfg["manipulator"]["position_only_ik"] is True


def test_moveit_servo_topics_relative():
    """Regression guard for the /planning_scene namespace leak."""
    cfg = load_yaml("config/moveit_servo.yaml")
    assert cfg["monitored_planning_scene_topic"] == "planning_scene"
    assert cfg["joint_topic"] == "joint_states"


def test_moveit_controllers_have_required_actions():
    """MoveIt's controller names must match ros2_control's."""
    cfg = load_yaml("config/moveit_controllers.yaml")
    controllers = cfg["moveit_simple_controller_manager"]["controller_names"]
    assert set(controllers) == {"manipulator_controller", "gripper_controller"}


def test_gripper_uses_follow_joint_trajectory():
    """JTC gripper requires FollowJointTrajectory; flipping to GripperCommand
    breaks joy2servo's direct trajectory publish path."""
    mscm = load_yaml("config/moveit_controllers.yaml")["moveit_simple_controller_manager"]
    assert mscm["gripper_controller"]["type"] == "FollowJointTrajectory"
    assert mscm["gripper_controller"]["action_ns"] == "follow_joint_trajectory"


def test_ompl_manipulator_planner_configured():
    """At least one OMPL planner for the manipulator group."""
    planners = load_yaml("config/ompl_planning.yaml")["ompl"]["manipulator"]["planner_configs"]
    assert len(planners) >= 1
    assert "RRTConnectkConfigDefault" in planners


def test_joint_limits_drops_passive_joint():
    """gripper_right_joint is <passive_joint>; stale limits caused confusion before."""
    cfg = load_yaml("config/joint_limits.yaml")
    assert "gripper_right_joint" not in cfg["joint_limits"]
    for joint in ["gripper_left_joint", "joint1", "joint2", "joint3", "joint4"]:
        assert joint in cfg["joint_limits"], f"Missing joint limits for {joint}"


def _srdf_close_value(taper_jaws: str) -> float:
    """Render the SRDF through xacro and return the gripper Close position.

    MoveItConfigsBuilder loads the SRDF via load_xacro(), so the file carries a
    taper_jaws argument even though it is named .srdf. Rendering it here is the
    only way to see what MoveIt will actually receive.
    """
    import subprocess
    import xml.etree.ElementTree as ET

    out = subprocess.run(
        ["xacro", share("config/rosbot_xl.srdf"), f"taper_jaws:={taper_jaws}"],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert out.returncode == 0, out.stderr
    root = ET.fromstring(out.stdout)
    for state in root.findall("group_state"):
        if state.attrib["name"] == "Close":
            return float(state.find("joint").attrib["value"])
    raise AssertionError("no Close group_state in rendered SRDF")


def _gripper_lower(use_sim: str) -> float:
    """The gripper's actual lower stop in the rendered URDF."""
    import subprocess
    import xml.etree.ElementTree as ET

    from ament_index_python.packages import get_package_share_directory

    urdf = os.path.join(
        get_package_share_directory("rosbot_description"), "urdf", "rosbot_xl.urdf.xacro"
    )
    out = subprocess.run(
        ["xacro", urdf, f"use_sim:={use_sim}", "configuration:=manipulation_pro"],
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert out.returncode == 0, out.stderr
    root = ET.fromstring(out.stdout)
    for joint in root.iter("joint"):
        if joint.get("name") == "gripper_left_joint":
            return float(joint.find("limit").get("lower"))
    raise AssertionError("no gripper_left_joint in rendered URDF")


@pytest.mark.parametrize(
    "taper_jaws,use_sim", [("false", "false"), ("true", "true")]
)
def test_srdf_close_state_reaches_the_gripper_stop(taper_jaws, use_sim):
    """Close must close, on whichever target is built.

    body.xacro moves the gripper's lower stop to -0.023 when the simulation
    only claw is rendered and leaves it at -0.010 otherwise. A single hardcoded
    Close value cannot satisfy both: -0.023 is out of bounds on hardware and
    setJointValueTarget rejects it, while -0.009 leaves the simulated jaws
    28 mm apart, wide enough for the 26 mm cork to slip through. That is what
    this guards, because the two files drifted apart once already.
    """
    close = _srdf_close_value(taper_jaws)
    lower = _gripper_lower(use_sim)

    assert close >= lower, (
        f"SRDF Close {close} is below the joint's lower stop {lower}; "
        "setJointValueTarget will reject it"
    )
    assert close - lower <= 0.002, (
        f"SRDF Close {close} does not reach the stop {lower}; "
        f"the jaws stay {1000 * (close - lower):.0f} mm apart"
    )
