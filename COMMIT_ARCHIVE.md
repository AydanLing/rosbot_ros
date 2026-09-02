# Commit message archive (fork commits only)

Original messages for the commits on this branch above the fork point
160825fee15d59cb20b0e976c1c95c11b14df078. Upstream history below that point is untouched.

## fbbb372  2026-09-02  Hopper down to 150 mm for arm reach margin

A further pickup standoff lowers the arm's ceiling, and both carry stages were
sitting exactly on their IK limits.

---
## 6a41d35  2026-08-26  Add the shuttlecock hopper, and move the lidar out of its way

Mounts the user's hopper mesh on the rear deck and relocates LDR06 to sit
beside it.

The mesh is authored in millimetres and its origin is not at its
footprint centre, so the joint offset subtracts (8.8, 32.8) mm to put the
solid over x -0.167..-0.067, flush with the chassis rear edge. Flush is
load-bearing: nothing overhangs, so robot_radius stays 0.22 in both
costmaps and in cmd_vel_guard, and the near face at -0.067 is 132 mm from
the arm mount, inside the reach band. Pushing it back clear of the
chassis would put it at 232 mm and out of reach.

Z is scaled to 0.60, 300 mm down to 180 mm. That is set by the arm, and
by the grip rather than by reach. The traverse to the hopper passes over
the rim, a higher rim needs a flatter tool to clear it, and a flatter
tool rotates the shuttlecock further from the attitude it was grasped in.
30 degrees of rotation carried 2/2 and 90 degrees dropped 2/2. 180 mm at
a 45 degree traverse sits between them and carried 2/2.

The lidar sat exactly where the hopper now is. It went forward of the arm
first, which put it inside the grasp volume, and inboard from there,
which put it under the carry path. It is now at (-0.117, +0.090, +0.02)
on cover_link: the hopper leaves 85 mm each side, the body spans
+0.055..+0.125 inside the 0.135 half-width, and the carry corridor is
clear. The cost is coverage, and it is real rather than cosmetic, because
AMCL localises off this: the hopper blanks a sector to -y and the arm
blanks the front, leaving a wedge toward +y.

The SRDF gains hopper_link pairs. It predates the hopper, so MoveIt
treated the gripper entering the mouth as a self collision and refused
every deposit. Only the links that must enter are listed; the upper arm
and body stay checked so the arm still cannot swing through it.

Not yet re-validated after the lidar move: scan_self_filter's body
rectangle and the costmap obstacle bands were both measured with the beam
at z 0.234, and the AMCL map was built from scans at that height.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## b6db368  2026-08-25  Give the gripper a realistic grip force; leave arm speed alone

gripper_left_joint is prismatic, so its effort limit is a force in
newtons, not a torque. It was 1 N. That is roughly the weight of a 100 g
object, and mid-pick the joint was measured sitting at 100% of it.

Raised to 10 N, which is the range an OpenMANIPULATOR-X gripper actually
produces through its linkage. Worth being precise about what this did and
did not fix: the jaws still stall at exactly -0.0136 against a -0.0230
target, the same figure as before, because that stall is the jaws closing
onto the cork and not a force ceiling. What changed is that the arm is
now loaded by an object it is actually holding -- joint3 went from 8% of
its effort limit to 78%, joint2 from 15% to 38%, joint4 from 5% to 52%.

The arm servos were never the constraint. Peak effort across joint1 to
joint4 during a pick was 4%, 15%, 8% and 5% of the 4.1 Nm available, so
raising servo_effort would change nothing.

Neither is arm speed. The 0.1 scaling factors look like an obvious
culprit and are not: measured on one bench, a shuttlecock laid on its
side at (0.217, 0),

    0.1  / 0.1   ->  19.8 s, PASS
    0.5  / 0.35  ->  19.5 s, FAIL (knocked object aside)

0.3 s is noise and the faster arm disturbs the object on the descent, so
the factors stay at 0.1 with the measurement recorded next to them.
Individual trajectory executions already finish in 0.21 to 0.36 s. The
pick's 20 s is mostly two failed executions at the start, ABORTED then
TIMED_OUT with a 1 s backoff after each, plus a fixed ~0.35 s between
every step. That is where the time is.

Gripper max_velocity in joint_limits goes 0.10 to 0.30 m/s because the
joint was measured at 0.245, so the old figure was describing the
hardware wrongly rather than constraining it.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 7bbfc8c  2026-08-23  Gate moveit_servo behind use_servo, default off

Nothing in the autonomous stack drives servo. Measured on the badminton
court with no trial running, servo_node sat at 64% of a core while
/servo_node/delta_twist_cmds had zero publishers, and the only publisher
on delta_joint_cmds was joy2servo itself. That is a teleop path, so it
cost most of a core to serve a gamepad nobody was holding.

On this 8-core machine that mattered. The stack was at load 27.6 with a
real-time factor of 0.54, and nav2 was missing its control deadlines and
timing out on long goals. With servo off and a clean start, load is 14.5
and RTF is back to ~0.96.

Set use_servo:=True for gamepad teleop.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 12dc004  2026-08-22  Remove the tapered-jaw geometry that no longer renders

The taper_pad and taper_spine macros described the original 18 deg pitched
pads with a 120 deg included V. That design was superseded by the user's CAD
claw, which renders from Gripper-Left.stl / Gripper-Right.stl with box
collision proxies, and the macros were left behind: both defined, neither
called, along with the eight properties only they used.

Checked before deleting rather than assumed. Every one of cone_half_angle,
v_half_angle, pad_x, pad_t, pad_z, pad_centre_x, pad_split and pad_inset had
ZERO references outside the two dead macros. taper_on is different, with 5 live
uses (it still selects gripper_lower), so it stays.

47 lines gone. All four render paths still produce a URDF -- manipulation and
manipulation_pro, use_sim true and false -- and colcon test rosbot_description
passes 31 tests, 0 failures.

Worth naming and not fixing here: the taper_jaws argument no longer describes
what it does. It renders the CAD claw; only the name survives from the V-pad
design. Renaming it is a public argument change that would need README and
rosbot_xl.urdf.xacro updated in step, so it is left for a deliberate pass
rather than folded into a deletion commit.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 8ed944c  2026-08-22  Give servo and dock the SRDF mapping too, not just move_group

981afd7 made the gripper's Close named state track the rendered joint limit,
but wired the mapping into move_group.launch.py alone. That fixed move_group's
planning scene and nothing else.

servo.launch.py and dock.launch.py each construct their own
MoveItConfigsBuilder and pass moveit_config.robot_description_semantic to their
nodes, so joy2servo and dock held an unmapped copy and kept resolving
setNamedTarget("Close") to -0.009. Those are the two call sites that actually
use the named state (joy2servo.cpp:121, dock.cpp:35), so in practice the fix
had not reached any real consumer: teleop and dock still closed to a 28 mm gap
and leaked the 26 mm cork, which is the exact failure the -0.023 stop exists to
prevent. grab_sequence's automated pick was never affected, since it publishes
-0.023 to gripper_controller/joint_trajectory directly.

Both now take a use_sim argument and override robot_description_semantic with
the same Command(xacro) substitution, defaulting to the hardware-safe -0.009 so
a caller that passes nothing keeps today's behaviour.

ParameterValue(value_type=str) is carried across deliberately. Without it
launch_ros YAML-parses the substitution result to infer a type, and the
rendered SRDF opens with "<!--GROUPS: ...", which YAML reads as a mapping key
and raises ScannerError, aborting the launch. That cost a debugging pass the
first time; the comment now travels with each copy.

This is the third copy of the same 20-line override. It would be better as a
shared helper in rosbot_moveit, but launch files importing a sibling module
needs sys.path handling that is its own small mess, so the duplication is
deliberate and each copy points at the others.

rviz/home/setup_assistant still build unmapped configs and are left alone: none
of them command the gripper.

colcon test rosbot_moveit: 15 tests, 0 failures. All three launch descriptions
build (servo 4 entities, dock 3, move_group 2). Not exercised in a running
simulator.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 7ba8de9  2026-08-21  Type the SRDF parameter explicitly; the override aborted the launch without it

981afd7 overrode robot_description_semantic with a Command(xacro) substitution
so the gripper's Close state matches the rendered joint limit. That change was
never launched, only unit-tested, and it breaks the manipulator bringup.

launch_ros YAML-parses a substitution's result to infer the parameter type. The
rendered SRDF opens its comment block with

    <!--GROUPS: Representation of a set of joints and links...

and YAML reads "GROUPS:" as a mapping key, so yaml.safe_load raises
ScannerError and the launch aborts before Gazebo comes up. Confirmed directly:
piping the rendered SRDF through yaml.safe_load reproduces the ScannerError.

Wrapping it in ParameterValue(..., value_type=str) skips the inference.

Worth flagging for anyone copying the pattern: the URDF override sitting a few
lines above uses a bare Command and works, but only because its render happens
to contain no "key: value" text. That is luck. Any substitution carrying
arbitrary XML should be typed explicitly.

generate_launch_description() now builds; the SRDF still renders -0.009 by
default and -0.023 with taper_jaws:=true, and the rosbot_moveit tests pass.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 981afd7  2026-08-20  Tie the MoveIt Close named state to the gripper stop that is rendered

The SRDF hardcoded gripper_left_joint: -0.009 while body.xacro moves the
lower stop to -0.023 whenever the simulation-only CAD claw is rendered. One
static value cannot serve both targets:

  hardware (use_sim=false)  lower = -0.010   -0.023 is OUT OF BOUNDS;
                                             setJointValueTarget rejects an
                                             out-of-bounds named state, which
                                             would break the gripper for teleop
  simulation (use_sim=true) lower = -0.023   -0.009 leaves the jaws 28 mm
                                             apart, wide enough for the 26 mm
                                             cork that the -0.023 stop exists
                                             to trap

MoveItConfigsBuilder loads the SRDF through load_xacro() (moveit_configs_
builder.py, robot_description_semantic), so the file accepts a taper_jaws
argument despite its .srdf extension and needed no rename -- which also keeps
conftest.py's raw ET.parse working, since the existing tests read state names
and groups rather than values.

The argument DEFAULTS to the hardware-safe -0.009, so every caller that passes
no mapping keeps exactly today's behaviour. move_group.launch.py gained a
use_sim argument and overrides robot_description_semantic with a Command(xacro)
substitution -- the same pattern, and for the same reason, as the URDF override
already sitting immediately above it. manipulator.yaml passes use_sim through.

Guard: test_srdf_close_state_reaches_the_gripper_stop renders BOTH the SRDF and
the URDF for each target and asserts the named state reaches the joint's lower
stop without exceeding it. Mutation-verified: restoring the hardcoded -0.009
fails the simulation case, and restoring the conditional passes it. The two
files drifted apart once already, which is what this exists to catch.

Scope of the bug, since it is narrower than it first looked: joy2servo.cpp:121
and dock.cpp:35 reach the state via setNamedTarget("Close") and are the paths
affected. grab_sequence's automated pick is NOT affected -- grasp_ball.py calls
its own local set_gripper("Close") helper, which publishes -0.023 straight to
gripper_controller/joint_trajectory and never consults the SRDF.

Known incomplete: servo.launch.py and dock.launch.py each build their own
MoveItConfigsBuilder without the mapping, so nodes launched from them still
hold a hardware-default semantic copy. Only move_group's planning scene is
corrected here. Recorded in MANIPULATOR.md.

Not verified in simulation: another agent holds the simulator. colcon test
passes (rosbot_moveit 15, rosbot_controller 22, 0 failures) and both xacro
renders were checked by hand, but no move_group has actually consumed this.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 92bd4c1  2026-08-20  docs(manipulator): document the claw, the chassis moves, and their HW impact

The branch shipped three commits of manipulator work with only the first one
documented. The CLAUDE.md section 7 checklist is now complete for all three.

MANIPULATOR.md: the "Tapered V jaws" description was stale — the 18 deg pads
and 120 deg V were superseded by the user-CAD claw, with a generated box
collision proxy in claw_collision.xacro. Rewritten to match what actually
renders, including the -0.023 finger stop, the measured 84.0 mm open /
0.0 mm closed, and the fact that MoveIt's SRDF Close state is still -0.009 so
only direct JTC commands reach the new stop.

New "Arm mount and lidar placement" section, because two changes on this branch
are NOT gated on use_sim and reach the physical robot: the arm mount moved
-0.11 -> +0.065 on cover_link, and LDR06 moved to -0.125 0.0 0.07 in both
manipulation*.yaml, which hardware reads too. Hardware needs the arm re-bolted
forward and a 70 mm lidar riser to match the URDF. Also records the two open
items: the MoveIt collision matrix was never regenerated after the mount move,
and a measured +-5 deg forward blind sector remains.

ARCHITECTURE.md: claw_collision.xacro is a generated file whose generator lives
in a sibling repo, and the lidar and arm mount positions are coupled — both are
inter-package contracts a reader can otherwise only learn by breaking them.

README.md: new "Derived arguments" table for wrist_roll / taper_jaws / the
manipulator.yaml use_sim arg. They are wired from use_sim rather than passed on
the command line, so listing them in the main table would misrepresent them.

Tests: the jaw-gap computation is lifted out of
test_taper_jaws_open_wider_than_the_target into a shared _jaw_gap helper and
made signed, so jaws meeting reads as 0.0 rather than as a large positive
distance. Two guards added:

* test_claw_closes_past_the_cork — the stock -0.010 stop bottoms out at exactly
  26 mm, the cork diameter. Threshold is 20 mm rather than 26 mm because at
  26 mm the regression passes by one float ulp; verified by mutation.
* test_lidar_sits_behind_the_arm_mount — ties the lidar pose to the arm mount
  in cover_link frame and derives the base-column top from the rendered
  collision cylinder. Runs at both use_sim values, since neither position is
  simulation-gated.

colcon build + colcon test clean: 75 tests, 0 failures (was 70). pre-commit
could NOT be run — neither pre-commit nor black is installed in this
environment and there is no pip, so formatting is unverified.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 374459d  2026-08-16  feat(manipulator): fit the CAD claw, and let the jaws close fully

Replaces the tapered V pads with the CAD claw, as a collision proxy generated
from the exported STLs: the mesh is kept for the visual, while collision is
axis-aligned boxes covering the same solid. A single mesh collision would be
run through a convex hull by the physics engine, which fills in the claw's
concave pocket and removes the feature that does the gripping. Boxes are also
all axis-aligned, which avoids the rotated-corner trap that made the previous
jaws open to 34.8 mm when their nominal thickness suggested far more.

The finger travel now runs to -0.023 instead of -0.010. The claw's grip
surfaces sit 2 mm outboard of the finger frame, so the gap is
2 x (0.021 + q + 0.002) and the old stop bottomed out at 26 mm, wide enough for
the shuttlecock's 26 mm cork to slip straight through. Only the lower stop
moves, so the open gap is unchanged at 84 mm and still clears the 65 mm skirt.
Gated with the rest of the claw, so the stock hardware jaws keep their original
travel.

Measured on the finished geometry: 84.0 mm open, 0.0 mm closed.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 5cabf92  2026-08-14  fix(manipulator): widen tapered jaws, move arm to chassis front, relocate lidar

The tapered jaws could never close on the target. Their real opening was
34.8 mm against a 65 mm shuttlecock skirt, so every approach was a collision
and the object was knocked aside rather than gripped. The gap had been sized
from 2*(0.021 + q - inset), which counts only the pad's 5 mm thickness; each
pad is rotated 60 deg about z and 18 deg about x, and those rotations swing its
30 mm and 75 mm dimensions into the closing direction, moving the corners about
20 mm further in. pad_inset moves to -0.019, giving 74.8 mm open and 22.8 mm
closed. The backing spine moves outboard with the pads, having become the
narrowest point once they were widened.

test_taper_jaws_open_wider_than_the_target computes the opening from the
rendered pad boxes with their rotations applied, rather than trusting the inset,
and fails below 70 mm. It caught the spine regression on its first run.

The arm moves from -0.110 to +0.065 so its working zone is ahead of the robot
instead of behind it. The lidar moves from the centre to -0.125 and up by 0.07:
mounted ahead of a forward-working arm it sits inside the arm's swept volume,
and MoveIt reports gripper_left_link colliding with rplidar_link on the descent.
Raising it clears the arm's base column, which spans z 0.133..0.193.

Known gap: a +-5 deg blind sector remains straight ahead, from the arm's upper
links at the parked pose. Measured, not estimated. Parking the arm elsewhere
only relocates the notch. Clearing it needs either a lateral lidar offset, which
moves the blind spot off the forward axis, or a mast above the arm's ~0.45 m
envelope. Simulation only; the hardware path is unaffected.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---
## 314053a  2026-08-12  feat(manipulator): raise arm joint effort, add sim-only wrist roll + tapered jaws

Arm joints 1-4 move from a placeholder effort of 1 to 4.1 Nm (servo_effort),
the XM430-W350 stall torque at 12 V. This affects hardware as well: at 1 Nm
joint2 could not hold the arm out and settled ~0.2 rad short of any commanded
reach-down pose, leaving the gripper ~30 mm high. The prismatic gripper joints
keep their original value; 15 N was tried and did not help the jaws close
against contact, so it was reverted.

Two additions are gated on use_sim through new wrist_roll / taper_jaws xacro
args, because the physical OpenMANIPULATOR-X is 4-DOF and
dynamixel_hardware_interface is pinned to number_of_joints=5:

* joint5 wrist roll between link5 and the gripper. Rolling about the tool axis
  does not move the grasp point, so joints 1-4 kinematics are untouched and the
  joint is deliberately kept out of the MoveIt manipulator group and the SRDF.
  It runs on its own wrist_roll_controller rather than as a fifth joint on
  manipulator_controller, which would fail to claim its interfaces on the
  4-DOF hardware. Its ros2_control is a separate WristRollSystem block so the
  upstream open_manipulator_description macro does not have to be forked.

* Tapered V jaws replacing the stock flat palms: two pads per jaw pitched 18
  degrees, splayed into a 120 degree included V. Collision geometry is boxes on
  purpose, since the physics engine takes a convex hull of mesh collisions,
  which fills the V notch in and removes the feature that does the gripping.

Guarded by test_wrist_roll_and_taper_jaws_are_simulation_only and
test_taper_jaw_collisions_are_convex_primitives; the existing xacro matrix
never varied use_sim, so this gating had no coverage before.

Co-Authored-By: Claude Opus 5 <noreply@anthropic.com>

---

## SHA mapping (original -> rewritten)

| before | after | subject |
|---|---|---|
| `fbbb372d3` | `3da538826` | Hopper down to 150 mm for arm reach margin |
| `6a41d35d4` | `978d69cff` | Add the shuttlecock hopper, and move the lidar out of its way |
| `b6db36824` | `b19fc94c8` | Give the gripper a realistic grip force; leave arm speed alone |
| `7bbfc8c3a` | `7fa623ba7` | Gate moveit_servo behind use_servo, default off |
| `12dc00473` | `1c49efcab` | Remove the tapered-jaw geometry that no longer renders |
| `8ed944cb9` | `377dfef91` | Give servo and dock the SRDF mapping too, not just move_group |
| `7ba8de9d3` | `346907cda` | Type the SRDF parameter explicitly; the override aborted the launch without it |
| `981afd767` | `0996eb8e6` | Tie the MoveIt Close named state to the gripper stop that is rendered |
| `92bd4c119` | `c930eca04` | docs(manipulator): document the claw, the chassis moves, and their HW impact |
| `374459d80` | `cc82d8613` | feat(manipulator): fit the CAD claw, and let the jaws close fully |
| `5cabf927b` | `467655071` | fix(manipulator): widen tapered jaws, move arm to chassis front, relocate lidar |
| `314053a32` | `45b201cc7` | feat(manipulator): raise arm joint effort, add sim-only wrist roll + tapered jaws |
