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

import itertools
import math
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


def _process_xacro_sim(use_sim, configuration="manipulation_pro"):
    """A manipulation URDF at a chosen use_sim, which the default matrix does
    not vary (it always renders the hardware path)."""
    share = get_package_share_directory("rosbot_description")
    xacro_path = os.path.join(share, "urdf", "rosbot_xl.urdf.xacro")
    doc = xacro.process_file(
        xacro_path,
        mappings={
            "mecanum": "True",
            "configuration": configuration,
            "use_sim": use_sim,
        },
    )
    return doc.toxml()


# Simulation-only arm additions, and the stock geometry they replace.
_SIM_ONLY_ARM = ['name="joint5"', 'name="link5_roll"', "WristRollSystem"]
_STOCK_JAW_MESHES = ["gripper_left_palm", "gripper_right_palm"]


def test_wrist_roll_and_taper_jaws_are_simulation_only():
    """joint5 and the custom claw exist only when use_sim is true.

    The physical OpenMANIPULATOR-X is 4-DOF with no roll servo, and
    dynamixel_hardware_interface is pinned to number_of_joints=5 (four arm
    joints plus the gripper), so a fifth arm joint on the hardware path has
    nothing to drive it. See CLAUDE.md §9 (2026-05-19) for the earlier attempt
    to add a joint there that the driver rejected.
    """
    sim = _process_xacro_sim("True")
    hw = _process_xacro_sim("False")

    missing = [tag for tag in _SIM_ONLY_ARM if tag not in sim]
    assert not missing, "simulation URDF lost its wrist-roll additions: " + ", ".join(missing)

    leaked = [tag for tag in _SIM_ONLY_ARM if tag in hw]
    assert not leaked, (
        "wrist roll leaked into the hardware URDF: "
        + ", ".join(leaked)
        + ". The 4-DOF arm cannot drive joint5; keep it gated on use_sim."
    )

    # The claw replaces the stock palm meshes rather than sitting alongside them.
    for mesh in _STOCK_JAW_MESHES:
        assert mesh in hw, f"hardware URDF must keep the stock jaw mesh {mesh}"
        assert mesh not in sim, (
            f"stock jaw mesh {mesh} is still in the simulation URDF — the "
            "custom claw is meant to replace it, not sit alongside it."
        )


def test_taper_jaw_collisions_are_convex_primitives():
    """Jaw collision must stay boxes, never the claw mesh.

    The concave grip pocket is the whole point of the claw, and a mesh collision
    is run through a convex hull by the physics engine, which fills the pocket
    in and silently turns the wrap into a flat paddle.
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
                f"{link_name} uses a mesh collision; its convex hull fills in "
                "the claw's grip pocket. Use box primitives."
            )
            assert collision.find(".//box") is not None, (
                f"{link_name} collision is neither box nor mesh — the claw "
                "collision proxy expects boxes."
            )


def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _jaw_gap(root, finger_position):
    """Gap in metres between the two jaws at a given gripper_left_joint value.

    Measured off the *rendered* collision boxes with their origins and rpy
    applied, never off a nominal inset in the xacro. The first iteration of
    these jaws was sized from pad thickness alone; each pad was rotated 60 deg
    about z and 18 deg about x, which swung its 30 mm and 75 mm dimensions into
    the closing direction and pulled the corners ~20 mm inboard. The jaws opened
    to 34.8 mm against a 65 mm object and knocked it aside on every approach
    without raising any error. The current claw boxes are axis-aligned, so the
    rotation terms are a no-op today, but they keep this honest if that changes.

    The result is signed and taken from the innermost corner, so the two jaws
    meeting reads as 0.0 and overshoot reads negative.
    """
    joint = root.find('.//joint[@name="gripper_left_joint"]')
    assert joint is not None, "gripper_left_joint missing"
    mount_y = float(joint.find("origin").get("xyz").split()[1])

    link = root.find('.//link[@name="gripper_left_link"]')
    assert link is not None, "gripper_left_link missing"
    closest = None
    for collision in link.findall("collision"):
        box = collision.find(".//box")
        if box is None:
            continue
        half = [float(v) / 2 for v in box.get("size").split()]
        origin = collision.find("origin")
        offset_y = float(origin.get("xyz").split()[1])
        rot = _rpy_matrix(*[float(v) for v in (origin.get("rpy") or "0 0 0").split()])
        for signs in itertools.product((-1, 1), repeat=3):
            local = [s * h for s, h in zip(signs, half)]
            corner_y = sum(rot[1][k] * local[k] for k in range(3))
            half_gap = mount_y + finger_position + offset_y + corner_y
            closest = half_gap if closest is None else min(closest, half_gap)

    assert closest is not None, "no box collisions found on the jaw"
    return 2 * closest


def test_taper_jaws_open_wider_than_the_target():
    """The jaws must actually fit around a 65 mm shuttlecock skirt."""
    root = ET.fromstring(_process_xacro_sim("True"))
    joint = root.find('.//joint[@name="gripper_left_joint"]')
    opening = _jaw_gap(root, float(joint.find("limit").get("upper")))
    assert opening > 0.070, (
        f"claw opens to only {1000 * opening:.1f} mm, which cannot clear the "
        "65 mm shuttlecock skirt. Check the rendered collision-box corner "
        "extents, not the nominal pad thickness."
    )


def test_claw_closes_past_the_cork():
    """The jaws must close far enough to trap a 26 mm shuttlecock cork.

    The stock lower stop of -0.010 bottoms out at a 26 mm gap, exactly wide
    enough for the cork to slip straight through, which is how the object kept
    escaping a nominally successful grasp. gripper_lower drops to -0.023 under
    taper_jaws, which brings the claws to touching. Hardware keeps the stock
    travel, since it also keeps the stock palms.

    The threshold is 20 mm, not 26 mm: matching the cork diameter exactly leaves
    no bite, and the stock stop lands on 26 mm to within floating-point noise,
    so a 26 mm bound would pass on the very geometry this guards against.
    """
    sim = ET.fromstring(_process_xacro_sim("True"))
    sim_joint = sim.find('.//joint[@name="gripper_left_joint"]')
    closed = _jaw_gap(sim, float(sim_joint.find("limit").get("lower")))
    assert closed < 0.020, (
        f"claw only closes to {1000 * closed:.1f} mm; a 26 mm cork needs a real "
        "bite, not a gap it can slip through. Check gripper_lower in body.xacro."
    )

    hw = ET.fromstring(_process_xacro_sim("False"))
    hw_joint = hw.find('.//joint[@name="gripper_left_joint"]')
    hw_lower = float(hw_joint.find("limit").get("lower"))
    assert hw_lower == pytest.approx(-0.010), (
        f"hardware finger travel changed to {hw_lower}; the extra travel is for "
        "the simulated claw only and the stock palms would drive into contact."
    )

    # Only the lower stop moves — the open gap must be unaffected.
    sim_upper = float(sim_joint.find("limit").get("upper"))
    hw_upper = float(hw_joint.find("limit").get("upper"))
    assert sim_upper == pytest.approx(hw_upper), "open stop must not differ between sim and HW"


@pytest.mark.parametrize("configuration", ["manipulation", "manipulation_pro"])
@pytest.mark.parametrize("use_sim", ["True", "False"])
def test_lidar_sits_behind_the_arm_mount(configuration, use_sim):
    """LDR06 placement is coupled to the arm mount, and not simulation-gated.

    The arm works forward from +0.065 on cover_link. A lidar ahead of it sits
    inside the swept volume — MoveIt reports gripper_left_link colliding with
    rplidar_link on the descent — so it moved behind the arm, and up by 0.07 so
    the beam clears the top of the arm's base column. Both numbers live in the
    components yaml, which hardware reads too, so this runs at both use_sim
    values on purpose. Moving one without the other silently reintroduces the
    collision or the blind sector. See MANIPULATOR.md.
    """
    root = ET.fromstring(_process_xacro_sim(use_sim, configuration))

    def origin_on_cover(child_link):
        for joint in root.iter("joint"):
            child = joint.find("child")
            if child is None or child.get("link") != child_link:
                continue
            assert joint.find("parent").get("link") == "cover_link", (
                f"{child_link} no longer hangs directly off cover_link; this "
                "test compares the two in that frame."
            )
            return [float(v) for v in joint.find("origin").get("xyz").split()]
        raise AssertionError(f"no joint produces {child_link}")

    lidar = origin_on_cover("rplidar_link")
    arm = origin_on_cover("link1")

    assert lidar[0] < arm[0], (
        f"lidar x {lidar[0]} is not behind the arm mount x {arm[0]}; a "
        "forward-working arm sweeps over anything mounted ahead of it."
    )
    assert lidar[0] > -0.167, f"lidar x {lidar[0]} is outside the chassis rear edge of -0.167"

    # Top of the arm's base column, in the same cover_link frame.
    column = root.find('.//link[@name="link1"]/collision')
    column_z = float(column.find("origin").get("xyz").split()[2])
    column_top = arm[2] + column_z + float(column.find(".//cylinder").get("length")) / 2
    assert lidar[2] > column_top, (
        f"lidar z {lidar[2]} is not above the arm base column top {column_top:.4f}; "
        "the column blanks roughly +-10 deg dead ahead. On hardware this "
        "clearance is a physical riser."
    )
