# ROSbot XL Manipulation

> [!WARNING]
> **Limitations**
>
> 1. **Before starting driver!** Make sure the manipulator is **undock**, manipulator is **away from a collision** (does not rest on robot objects) and **joints are away from its position limits** (e.g. one of the joints is started from extreme position).
> 2. Controlling MoveIt and via the joystick are two independent processes. You should not send commands to both of these services at the same time.
> 3. When the power supply is lost, the robot loses momentum and falls by inertia. Therefore, you should hold the manipulator when the power is cut off, or call the docking node `ros2 launch rosbot_moveit dock.launch.py` (pass `namespace:=$ROBOT_NAMESPACE` when the robot was launched in a namespace so `dock` reaches the namespaced `move_group`) or press `RT` + `Back` buttons on gamepad. The launch wrapper injects `robot_description_kinematics` etc. so the `MoveGroupInterface` inside `dock` does not log `No kinematics plugins defined`; the executable can still be run bare with `ros2 run rosbot_moveit dock --ros-args -r __ns:=/$ROBOT_NAMESPACE` if you only need a quick test.
> 4. The manipulator does not analyze collisions with the antenna, to improve the range of the manipulator's movements. It's good practice to position the antenna horizontally on the physical robot.
> 5. In the event of overload, loss of communication or sudden stopping of the manipulator process (e.g. during reboot), some joints may not receive the command to stop operation. This may prevent re-establishing communication. In such a case, it will be necessary to **reset the power supply**.

Below is a handful of the most important information for the ROSbot Manipulation/Manipulation PRO package.

## Activation

By default, the arm remains in an idle state after the driver is launched. To activate it, execute:

```bash
sudo rosbot.arm-activate # if you are using snap
ros2 run rosbot_controller arm_control active # if you are using local build
```

You can change the driver's default behavior using the `arm_activate` argument.

## Control

After starting, the manipulator should be in the Home position after a few seconds. Now you can control ROSbot XL and OpenMANIPULATOR-X using a gamepad or RViz.

> [!NOTE]
> The instructions presented are for the Jazzy version. You can check the difference in versions by changing the branch in the [rosbot_ros/MANIPULATOR.MD](https://github.com/husarion/rosbot_ros/blob/jazzy/MANIPULATOR.md) repository.

### Gamepad

After running the ROSbot XL Manipulation Package, you should be able to control the manipulator. The easiest way to move the manipulator is to connect a gamepad and steer the robot. The graphic below shows how to steer the manipulator using a gamepad.

![gamepad_manipulation](.docs/gamepad_manipulation.drawio.png)

Drive controls (`cmd_vel`) are configured in [`rosbot_joy/config/config.yaml`](rosbot_joy/config/config.yaml). Manipulator gamepad mappings are hardcoded in [`rosbot_moveit/src/joy2servo.cpp`](rosbot_moveit/src/joy2servo.cpp) (`enum Axis` / `enum Button`).

Everything below runs **only while the dead-man trigger `RT` is held** (right trigger ≤ -0.3).

#### Mode toggle

| Button | Mode | Behaviour |
|---|---|---|
| **Y** | `Cartesian` **(default)** — XYZ in EE frame | `joy2servo` integrates stick velocity (`cartesian_linear_velocity` param, default `0.1 m/s`) into a target EE pose, runs KDL position-only IK in-process, and publishes the resulting joint velocities as **JointJog**. The IK is done in joy2servo because `moveit_servo`'s POSE/TWIST paths run a singularity guard on the full 6×N Jacobian which trips on any 4-DoF pose (see [moveit_msgs#185](https://github.com/moveit/moveit_msgs/issues/185)); the JointJog path does not run that guard. |
| **X** | `JointSpace` — per-joint | Each stick axis drives one joint directly. No IK. Useful as a low-level fallback for joints unreachable in Cartesian (e.g. joint4 wrist when EE is at limit). |

Both modes publish on `servo_node/delta_joint_cmds`. X / Y are XOR-ed — pressing both at once is a no-op.

#### JOINT_JOG axis map

| Stick | Joint |
|---|---|
| Left stick X | `joint1` |
| Left stick Y | `joint2` |
| Right stick X | `joint3` |
| Right stick Y | `joint4` (inverted: stick up = joint up) |

#### Cartesian axis map (delta interpreted in EE frame)

Each tick joy2servo reads the current EE pose, adds `stick * cartesian_linear_velocity * dt` in EE frame, runs KDL position-only IK on the result, and publishes the resulting joint velocities as JointJog. Sign convention: stick up / right = positive.

| Stick | Direction |
|---|---|
| Right stick Y | ±X (stick up = arm extends forward) |
| Right stick X | ±Y (stick right = sideways across the gripper) |
| Left stick Y | ±Z (stick up = perpendicular-up to the gripper) |
| Left stick X | unused |

Releasing the stick zeroes the joint velocities — the arm stops immediately at the current pose. IK failures (out-of-reach targets) are logged and skipped silently.

#### Combo actions (work in both modes)

| Combo | Action |
|---|---|
| `RT + Back` | Move to **Dock** pose (gripper Close → arm Dock, planned via OMPL through `MoveGroupInterface`) |
| `RT + Start` | Move to **Home** pose (arm Home → gripper Open) |
| `RT + RB` + `LT axis` | Analogue gripper — `LT` axis maps to `gripper_left_joint` position (`-0.009 m` … `+0.015 m`) |

#### Tunable parameters

```bash
ros2 run rosbot_moveit joy2servo --ros-args \
  -p cartesian_linear_velocity:=0.2 \
  -p cartesian_step_dt:=0.05 \
  -p cartesian_max_joint_velocity:=1.0
```

- `cartesian_linear_velocity` (m/s, default `0.1`) — EE linear speed when a stick is at full deflection
- `cartesian_step_dt` (seconds, default `0.05`) — per-tick integration step; should be ≥ servo's `publish_period` and ≈ joy autorepeat period (default `1/20 Hz = 0.05`)
- `cartesian_max_joint_velocity` (rad/s, default `1.0`) — uniform cap on the joint velocities produced by Cartesian-mode IK. Applied as a single scaling factor so the EE direction is preserved (just slower). Bounds how far the arm can travel per `collision_check_rate` tick — without it, KDL IK "branch jumps" near singularities can produce multi-rad/s spikes that overshoot `self_collision_proximity_threshold` before the collision check updates.

You may have noticed that the movement of the manipulator is slow, and the full capabilities of the manipulator are not fully utilized. This is a safety precaution to ensure that the collision checker effectively prevents the manipulator from bumping into the robot.
The dynamic limits of the manipulator have been tuned in order to provide a reliable collision prevention mechanism. While this setup should cover most situations, there is still a possibility of accidental contact with the robot or its sensors. Therefore, we advise you to remain aware of this potential risk when operating the manipulator.

### RViz Moveit

To move the manipulator using RViz, you need to build the code first. Then run:

```bash
ros2 launch rosbot_moveit rviz.launch.py
```

After that, RViz with the Moveit configuration will appear.

![moveit_manipulation](.docs/moveit_manipulation.png)

It is also possible to control the manipulator in the RViz using the *MotionPlanning* plugin. You can move the end effector by dragging the green marker and then click the `Plan & Execute` button to make the manipulator execute the motion. You may notice that the end effector doesn't follow the marker exactly, and to get Z rotation you have to use the blue ring - this is due to insufficient degrees of freedom of the manipulator, which causes some of the configurations to not be achievable.

> [!TIP]
> If you're not able to move the end effector in the Rviz, make sure that you have enabled the *Approx IK Solutions* option.
> If the manipulator moves too slowly, you can increase *Velocity Scaling* and *Accel. Scaling* up to `1.0`.

## Helpful Resources

### Resetting the Manipulator

In some situations it may be necessary to manually move the manipulator out of an invalid configuration.
To do it, first you will have to disable the torque of the manipulator, for example using the service.
On your ROSbot XL execute. **Hold the manipulator** while doing it, as it disables the torque and the manipulator can fall.

```bash
sudo rosbot.arm-disactivate # if you are using snap
ros2 run rosbot_controller arm_control inactive # if you are using local build
```

Now you can manually move the manipulator to the desired position and launch:

```bash
sudo rosbot.arm-activate # if you are using snap
ros2 run rosbot_controller arm_control active # if you are using local build
```

### Modifications

For more advanced purpose you may want to change default **manipulator position** or edit dynamixel **servo setting**. To do this it will be necessary to build [`rosbot_ros`](https://github.com/husarion/rosbot_ros/) and for:

1. Manipulator position
   - change position of the manipulator in the [`rosbot_xl.urdf.xacro`](https://github.com/husarion/rosbot_ros/blob/jazzy/rosbot_description/urdf/rosbot_xl.urdf.xacro)
   - regenerate collision matrix using [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

2. Servo setting
   Check/change some servo parameters in [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).

### Simulation-only arm features

Two additions exist **only** when the URDF is built with `use_sim:=True`. The
physical OpenMANIPULATOR-X is 4-DOF with no roll servo, and
`dynamixel_hardware_interface` is pinned to `number_of_joints=5` (four arm
joints plus the gripper), so neither can be driven on hardware. Both are gated
by xacro args (`wrist_roll`, `taper_jaws`) that
[`rosbot_xl.urdf.xacro`](rosbot_description/urdf/rosbot_xl.urdf.xacro) ties to
`use_sim`. Guarded by `test_wrist_roll_and_taper_jaws_are_simulation_only`.

**Wrist roll (`joint5`)** — a revolute joint between `link5` and the gripper,
turning about the tool axis, so it spins the jaws about their approach
direction without moving the grasp point. Position kinematics for joints 1-4
are therefore unchanged, and the MoveIt `manipulator` group and SRDF are
untouched: `joint5` is deliberately **not** part of that group. It runs on its
own `wrist_roll_controller`, commanded directly like the joystick gripper:

```bash
ros2 topic pub --once /wrist_roll_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [joint5], points: [{positions: [0.785], time_from_start: {sec: 1}}]}'
```

With the tool pointing straight down, `link5` x is `(0,0,-1)` and its y (the jaw
line) is the arm's pitch axis at bearing `joint1+90`, so `z = x × y` puts the
jaw's z at bearing `-joint1`, and rolling by `theta` about a downward axis turns
bearings by `-theta`:

```
jaw_z_bearing = -joint1 - theta
```

**Custom claw (`taper_jaws`)** — replaces the stock flat palms. The arg is still
named `taper_jaws` after the first iteration, which was two flat pads per jaw
pitched by 18° into a 120° included V, sized for a badminton shuttlecock's cone.
That shape is gone; what ships now is a user-supplied CAD claw:

- **Visual** — [`Gripper-Left.stl`](rosbot_description/meshes/rosbot_xl/claw/Gripper-Left.stl)
  and [`Gripper-Right.stl`](rosbot_description/meshes/rosbot_xl/claw/Gripper-Right.stl),
  mounted with a ±2 mm outboard offset and `rpy="-pi/2 0 pi/2"`.
- **Collision** — 32 left / 25 right axis-aligned boxes in
  [`claw_collision.xacro`](rosbot_description/urdf/open_manipulator/claw_collision.xacro),
  voxelised from those same STLs at 8 mm. **Generated, not hand-written.**
  Regenerate with `grab_sequence/scripts/mesh_to_collision.py` if the CAD changes;
  the header of the file says the same thing.

The jaw collision is **boxes, not a mesh** — the physics engine takes a convex
hull of mesh collisions, which fills the claw's concave grip pocket in and
silently turns the wrap into a flat paddle. That is the whole feature, so
`test_taper_jaw_collisions_are_convex_primitives` pins it.

Keeping the boxes axis-aligned also avoids the trap the first iteration fell
into. Those pads were rotated 60° about z and 18° about x, which swung their
30 mm and 75 mm dimensions into the closing direction and moved the corners
about 20 mm inboard of what the 5 mm pad thickness suggested. The jaws opened
to 34.8 mm against a 65 mm object and knocked it aside on every approach without
raising any error. `test_taper_jaws_open_wider_than_the_target` computes the
opening from the *rendered* boxes rather than from a nominal inset, and fails
below 70 mm.

**Finger travel.** With the claw on, `gripper_lower` moves from `-0.010` to
`-0.023`. The claw's grip surfaces sit 2 mm outboard of the finger frame, so the
gap is `2 × (0.021 + q + 0.002)`; the old stop bottomed out at 26 mm — exactly
wide enough for a 26 mm shuttlecock cork to slip through. Only the lower stop
moves, so the open gap is unchanged. Measured on the finished geometry:
**84.0 mm open, 0.0 mm closed.** The stock hardware jaws keep `-0.010`.

> [!NOTE]
> The MoveIt `Close` group state in
> [`rosbot_xl.srdf`](rosbot_moveit/config/rosbot_xl.srdf) is still
> `gripper_left_joint: -0.009` (a 28 mm gap). The new `-0.023` stop is reachable
> only by commanding `gripper_controller/joint_trajectory` directly, which is
> what `joy2servo` does. `joint_limits.yaml` has no explicit min/max for that
> joint, so planning picks the URDF limit up automatically.

> **Arm joint effort** was raised from a placeholder `1` to `4.1` N·m
> (`servo_effort` in [`body.xacro`](rosbot_description/urdf/open_manipulator/body.xacro)),
> the XM430-W350 stall torque at 12 V. At `1` N·m joint2 could not hold the arm
> out and settled ~0.2 rad short of any commanded reach-down pose, putting the
> gripper ~30 mm high. This affects **both** hardware and simulation. The
> prismatic gripper joints keep their original value; raising them to 15 N was
> tried and did not help close against contact, so it was reverted.

### Arm mount and lidar placement

> [!WARNING]
> Unlike the two features above, **these two changes are not gated on `use_sim`
> and apply to the physical robot as well.** They describe a chassis layout that
> differs from the stock ROSbot XL manipulation build, and hardware needs
> physical rework to match.

**Arm moved to the chassis front.** In
[`rosbot_xl.urdf.xacro`](rosbot_description/urdf/rosbot_xl.urdf.xacro) the
`open_manipulator_x` mount on `cover_link` moved from `-0.11 0.0 0.0` to
`0.065 0.0 0.0`, so the working zone is ahead of the robot instead of behind it.
On hardware the arm must actually be bolted at the front, or the URDF and the
robot disagree and every planned pose is off by 175 mm.

**Lidar moved back and up.** `LDR06` in
[`manipulation.yaml`](rosbot_description/config/rosbot_xl/manipulation.yaml) and
[`manipulation_pro.yaml`](rosbot_description/config/rosbot_xl/manipulation_pro.yaml)
moved from `-0.01 0.0 0.0` to `-0.125 0.0 0.07`. These component files are read
by both the hardware and the simulation URDF, so this applies to both.

- **Back to `-0.125`** because a forward-working arm sweeps over a
  centre-mounted lidar; MoveIt reports `gripper_left_link` colliding with
  `rplidar_link` on the descent. From the rear the arm never reaches over it.
  Still inside the chassis, whose rear edge is `-0.167`.
- **Up by `0.07`** so the beam clears the arm's base column, which spans
  `z 0.133..0.193` and otherwise blanked ±10° dead ahead. **On hardware this
  needs a riser of the same height** — without one the physical scan will not
  match the URDF.

> **Known gap, measured not estimated:** a ±5° blind sector remains straight
> ahead, cast by the arm's upper links at the parked pose. Parking the arm
> elsewhere only relocates the notch. Clearing it needs either a lateral lidar
> offset, which moves the blind spot off the forward axis, or a mast above the
> arm's ~0.45 m envelope. Relevant to nav2 and to the laser filter in
> [`rosbot_utils/config/rosbot_xl/config.yaml`](rosbot_utils/config/rosbot_xl/config.yaml).

> [!IMPORTANT]
> Moving the arm mount is exactly the case the "Manipulator position" note above
> covers: the MoveIt collision matrix should be regenerated. It has **not** been.
> [`rosbot_xl.srdf`](rosbot_moveit/config/rosbot_xl.srdf) still carries
> `link1 ↔ rplidar_link reason="Adjacent"` (plus `link2`/`link3` `Never`) from
> the old layout where the two were next to each other. Those pairs are now
> merely over-permissive rather than wrong, but the matrix no longer reflects
> the geometry.

### Troubleshooting

- **Gripper does not move** - it could be caused by the wrong initial position. Turn off the torque as described in the [**Resetting the manipulator**](/tutorials/ros-projects/rosbot-xl-openmanipulator-x/#resetting-the-manipulator). Now manually rotate the hub of the manipulator servo 180 degrees (if the connecting rod of the left finger was below the right finger's one, the hub should be rotated so that it is above). Now once again launch controllers.
- **Manipulator stopped moving** - it can be too close to collision or singularity (you can verify it by examining console logs). The easiest solution to this problem is to return the manipulator to the *Home* position (`RT` + `Start` button).
- **Manipulator stopped moving and *Start* button does not work** - if the manipulator still won't move, it could be already in the collision (you can verify it by examining console logs). In this case, follow the [**Resetting the manipulator**](/tutorials/ros-projects/rosbot-xl-openmanipulator-x/#resetting-the-manipulator) step and return it manually to some valid position. If nothing helps try restart power supply.
