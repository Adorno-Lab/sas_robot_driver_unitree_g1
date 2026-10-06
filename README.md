# sas_robot_driver_unitree_g1

SmartArmStack driver of the **Unitree G1 (29 DoF)** in high-level control: Unitree's locomotion controller moves
the legs, and `rt/arm_sdk` moves the waist and the arms.

The driver follows the legged robot driver standard of [sas_tools](https://github.com/Adorno-Lab/sas_tools/tree/new_generation) (branch `new_generation`): it
implements `LeggedRobotDriver` and runs with `LeggedRobotDriverROS`. A control package that uses
`LeggedRobotDriverClient` works with this robot and with any other legged robot that follows the standard.

## Limbs

The G1 is a floating base with five limbs. Each limb is a standard SAS robot driver on `sas_g1/g1_1/<limb>`.

| Limb | Joints | Accepts targets |
|---|---|---|
| `left_leg` | LeftHipPitch, LeftHipRoll, LeftHipYaw, LeftKnee, LeftAnklePitch, LeftAnkleRoll | never (moved by the locomotion controller) |
| `right_leg` | RightHipPitch, RightHipRoll, RightHipYaw, RightKnee, RightAnklePitch, RightAnkleRoll | never (moved by the locomotion controller) |
| `waist` | WaistYaw, WaistRoll, WaistPitch | in `STANDING` |
| `left_arm` | LeftShoulderPitch, LeftShoulderRoll, LeftShoulderYaw, LeftElbow, LeftWristRoll, LeftWristPitch, LeftWristYaw | in `STANDING` |
| `right_arm` | RightShoulderPitch, RightShoulderRoll, RightShoulderYaw, RightElbow, RightWristRoll, RightWristPitch, RightWristYaw | in `STANDING` |

- The joint names and their order are the ones of [unitree_drivers](https://github.com/Adorno-Lab/unitree_drivers)
  (`DriverUnitreeLowState::LIMB`), which follow the `G1JointIndex` of unitree_sdk2.
- The joint limits are taken from `g1_29dof_rev_1_0.urdf` of
  [unitree_ros](https://github.com/unitreerobotics/unitree_ros) (identical in `g1_29dof.urdf`).

## Modes and capabilities

| Mode | G1 behaviour |
|---|---|
| `IDLE` (initial mode) | Zero velocity. The arms keep their current state (arm control is not switched off, so they do not move back to Unitree's default pose). |
| `STANDING` | Arm control (`rt/arm_sdk`): the waist and the arms accept targets; the robot does not walk. |
| `WALKING` | Locomotion: the twist is accepted; the waist and the arms do not accept targets. |

- Every mode change first stops the robot (zero velocity).
- Only the twist is supported: `vx`, `vy`, and `wz` (yaw rate). The base height and the base orientation are not
  supported.

## Topics

| Prefix | Topics |
|---|---|
| `sas_g1/g1_1/` (base) | `get/info`, `get/status`, `get/imu`, `set/high_level_mode`, `set/target_twist`, `set/watchdog_trigger`, `set/shutdown` |
| `sas_g1/g1_1/<limb>/` | the standard SAS joint topics: `get/joint_states`, `get/joint_positions_min`, `get/joint_positions_max`, `set/target_joint_positions`, `set/shutdown` |

See the [topic standard of sas_tools](https://github.com/Adorno-Lab/sas_tools/tree/new_generation#topic-standard) for the message types,
the client roles, and the watchdog.

## Install

Requires ROS 2 Jazzy, [DQ Robotics](https://github.com/dqrobotics/cpp), the SmartArmStack packages (`sas_core`,
`sas_common`, `sas_robot_driver`), and:

- [sas_tools](https://github.com/Adorno-Lab/sas_tools), in the same workspace.
- [unitree_drivers](https://github.com/Adorno-Lab/unitree_drivers), installed in the system. It handles the DDS
  communication with the robot.

```shell
cd ~/ros2_ws
colcon build --packages-up-to sas_robot_driver_unitree_g1
```

## Usage

Run the driver in a different terminal window or tab. Be ready to close it, as it activates the real robot if the
connection is successful.

```shell
ros2 launch sas_robot_driver_unitree_g1 start_high_level_driver.py
```

| Launch argument | Default | Description |
|---|---|---|
| `domain_id` | `0` | DDS domain: `0` for the real robot, `1` for `unitree_mujoco` |
| `network_interface` | `eth0` | Network interface connected to the robot: `eth0` on the onboard computer, `lo` for `unitree_mujoco` |
| `control_level` | `high` | How the commands are sent; both levels read the state from `rt/lowstate`. `high`: through the locomotion controller and `rt/arm_sdk`. `low`: through `rt/lowcmd`, for every joint; **not implemented yet**, the driver stops with an error. |

The node name and namespace in the launch file (`g1_1` in `sas_g1`) define the topic prefix, `sas_g1/g1_1`. The
launch file also sets the node parameters `thread_sampling_time_sec` (0.002 s) and `twist_timeout_sec` (0.2 s).

To command the robot, use the clients of sas_tools with these launch parameters (see the
[example in sas_tools](https://github.com/Adorno-Lab/sas_tools/tree/new_generation#commanding-the-robot)):

```yaml
legged_robot_prefix: "sas_g1/g1_1"
limb_prefixes: ["sas_g1/g1_1/left_leg", "sas_g1/g1_1/right_leg", "sas_g1/g1_1/waist",
                "sas_g1/g1_1/left_arm", "sas_g1/g1_1/right_arm"]
```

## Known limitations

- `unitree_mujoco` only simulates the low-level interface (`rt/lowstate`, `rt/lowcmd`). In the simulator, the driver
  reads the joint states and the IMU, but it cannot walk or move the arms, since there is no locomotion service and
  no `rt/arm_sdk`.
- Without a locomotion service (e.g. no robot connected), stopping the driver waits for the pending RPC calls of the
  locomotion client of unitree_drivers.
