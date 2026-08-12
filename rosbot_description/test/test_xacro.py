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

"""xacro -> URDF for every (robot_model, mecanum, configuration) combo."""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

# rosbot_xl picks components yaml in rosbot_description/config/rosbot_xl/ by name.
ROSBOT_XL_CONFIGURATIONS = [
    "basic",
    "telepresence",
    "autonomy",
    "manipulation",
    "manipulation_pro",
    "custom",
]


def _process_xacro(robot_model, mecanum, configuration):
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", f"{robot_model}.urdf.xacro")
    mappings = {"mecanum": mecanum}
    if robot_model == "rosbot_xl":
        mappings["configuration"] = configuration
    doc = xacro.process_file(xacro_path, mappings=mappings)
    return doc.toxml()


def _params(model, mecanum, configurations):
    return [(model, mecanum, cfg) for cfg in (configurations or [None])]


_TEST_MATRIX = (
    _params("rosbot", "True", None)
    + _params("rosbot", "False", None)
    + _params("rosbot_xl", "True", ROSBOT_XL_CONFIGURATIONS)
    + _params("rosbot_xl", "False", ROSBOT_XL_CONFIGURATIONS)
)


@pytest.mark.parametrize("robot_model,mecanum,configuration", _TEST_MATRIX)
def test_xacro_produces_valid_urdf(robot_model, mecanum, configuration):
    urdf = _process_xacro(robot_model, mecanum, configuration)
    assert (
        "<link" in urdf
    ), f"No <link> in URDF for ({robot_model}, mecanum={mecanum}, {configuration})"
    assert (
        "<joint" in urdf
    ), f"No <joint> in URDF for ({robot_model}, mecanum={mecanum}, {configuration})"
    assert f'<robot name="{robot_model}"' in urdf, "Robot name tag missing or mismatched"


@pytest.mark.parametrize("configuration", ROSBOT_XL_CONFIGURATIONS)
def test_manipulator_presence_matches_configuration(configuration):
    """CLAUDE.md §8: configuration=manipulation* spawns the OpenMANIPULATOR-X."""
    urdf = _process_xacro("rosbot_xl", "False", configuration)
    has_manipulator = '"joint1"' in urdf or 'name="joint1"' in urdf
    if configuration.startswith("manipulation"):
        assert has_manipulator, (
            f"configuration='{configuration}' must include the manipulator macro "
            "(joint1 missing from URDF)"
        )
    else:
        assert not has_manipulator, (
            f"configuration='{configuration}' must NOT include the manipulator "
            "(joint1 present unexpectedly)"
        )


def test_gazebo_urdf_namespace_remappings():
    """Pins the gz_ros2_control <remapping> set — push_ros_namespace cannot
    reach this plugin (gz_sim hosts it outside the LaunchContext)."""
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", "rosbot_xl.urdf.xacro")
    doc = xacro.process_file(
        xacro_path,
        mappings={"mecanum": "False", "configuration": "basic", "use_sim": "True"},
    )
    urdf = doc.toxml()
    root = ET.fromstring(urdf)

    plugin = None
    for elem in root.iter("plugin"):
        if "gz_ros2_control" in (elem.get("filename") or ""):
            plugin = elem
            break
    assert plugin is not None, "gz_ros2_control-system plugin missing from URDF"

    actual_remaps = {r.text for r in plugin.iter("remapping") if r.text}

    expected_remaps = {
        "/diagnostics:=diagnostics",
        "/tf:=tf",
        "/tf_static:=tf_static",
        "/controller_manager/cleanup_controller:=controller_manager/cleanup_controller",
        "/controller_manager/configure_controller:=controller_manager/configure_controller",
        "/controller_manager/list_controllers:=controller_manager/list_controllers",
        "/controller_manager/list_controller_types:=controller_manager/list_controller_types",
        "/controller_manager/list_hardware_components:=controller_manager/list_hardware_components",
        "/controller_manager/list_hardware_interfaces:=controller_manager/list_hardware_interfaces",
        "/controller_manager/load_controller:=controller_manager/load_controller",
        "/controller_manager/reload_controller_libraries:=controller_manager/reload_controller_libraries",
        "/controller_manager/set_hardware_component_state:=controller_manager/set_hardware_component_state",
        "/controller_manager/switch_controller:=controller_manager/switch_controller",
        "/controller_manager/unload_controller:=controller_manager/unload_controller",
        "/controller_manager/activity:=controller_manager/activity",
        "/controller_manager/statistics:=controller_manager/statistics",
        "/controller_manager/introspection_data:=controller_manager/introspection_data",
    }
    missing = expected_remaps - actual_remaps
    assert not missing, (
        "gazebo.urdf.xacro <remapping> block is missing entries required for "
        "multi-robot namespace isolation: " + ", ".join(sorted(missing))
    )


def test_components_config_derived_from_configuration():
    """Soft-compat: components_config must derive from configuration arg
    (basic has no lidar; manipulation has rplidar_link)."""
    basic_urdf = _process_xacro("rosbot_xl", "False", "basic")
    manipulation_urdf = _process_xacro("rosbot_xl", "False", "manipulation")
    assert "rplidar_link" not in basic_urdf, (
        "basic configuration unexpectedly carries lidar — components_config "
        "derivation is reading the wrong yaml."
    )
    assert "rplidar_link" in manipulation_urdf, (
        "manipulation configuration is missing the lidar component — "
        "components_config did not derive from configuration. "
        "Did rosbot_xl.urdf.xacro lose its soft-compat property?"
    )


def _process_xacro_sim(use_sim):
    """manipulation_pro URDF at a chosen use_sim, which the default matrix does
    not vary (it always renders the hardware path)."""
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", "rosbot_xl.urdf.xacro")
    doc = xacro.process_file(
        xacro_path,
        mappings={
            "mecanum": "True",
            "configuration": "manipulation_pro",
            "use_sim": use_sim,
        },
    )
    return doc.toxml()


# Simulation-only arm additions, and the stock geometry they replace.
_SIM_ONLY_ARM = ['name="joint5"', 'name="link5_roll"', "WristRollSystem"]
_STOCK_JAW_MESHES = ["gripper_left_palm", "gripper_right_palm"]


def test_wrist_roll_and_taper_jaws_are_simulation_only():
    """joint5 and the tapered V jaws exist only when use_sim is true.

    The physical OpenMANIPULATOR-X is 4-DOF with no roll servo, and
    dynamixel_hardware_interface is pinned to number_of_joints=5 (four arm
    joints plus the gripper), so a fifth arm joint on the hardware path has
    nothing to drive it. See CLAUDE.md §9 (2026-05-19) for the earlier attempt
    to add a joint there that the driver rejected.
    """
    sim = _process_xacro_sim("True")
    hw = _process_xacro_sim("False")

    missing = [tag for tag in _SIM_ONLY_ARM if tag not in sim]
    assert not missing, (
        "simulation URDF lost its wrist-roll additions: " + ", ".join(missing)
    )

    leaked = [tag for tag in _SIM_ONLY_ARM if tag in hw]
    assert not leaked, (
        "wrist roll leaked into the hardware URDF: "
        + ", ".join(leaked)
        + ". The 4-DOF arm cannot drive joint5; keep it gated on use_sim."
    )

    # The tapered jaws replace the stock palm meshes rather than adding to them.
    for mesh in _STOCK_JAW_MESHES:
        assert mesh in hw, f"hardware URDF must keep the stock jaw mesh {mesh}"
        assert mesh not in sim, (
            f"stock jaw mesh {mesh} is still in the simulation URDF — the "
            "tapered V jaws are meant to replace it, not sit alongside it."
        )


def test_taper_jaw_collisions_are_convex_primitives():
    """Jaw collision must stay boxes, never a single notched mesh.

    The V notch is the whole point of the tapered jaw, and a mesh collision is
    run through a convex hull by the physics engine, which fills the notch in
    and silently removes the feature that does the gripping.
    """
    sim = _process_xacro_sim("True")
    root = ET.fromstring(sim)
    for link_name in ("gripper_left_link", "gripper_right_link"):
        link = root.find(f'.//link[@name="{link_name}"]')
        assert link is not None, f"{link_name} missing from simulation URDF"
        collisions = link.findall("collision")
        assert collisions, f"{link_name} has no collision geometry"
        for collision in collisions:
            assert collision.find(".//mesh") is None, (
                f"{link_name} uses a mesh collision; the convex hull of the "
                "tapered jaw fills in its V notch. Use box primitives."
            )
            assert collision.find(".//box") is not None, (
                f"{link_name} collision is neither box nor mesh — the taper "
                "geometry expects boxes."
            )
