# NAO One-Arm IK Web Teleoperator

This package provides a LeRobot teleoperator that controls exactly one NAO arm at a time from a browser-based 3D interface.

It uses Viser for the web UI, Pyroki for inverse kinematics, and a NAOqi robot plugin to send joint commands to a physical NAO. The rest of the body stays locked during IK optimization.

## Requirements

- A working LeRobot environment
- The dependencies from `requirements.txt`
- A loadable NAO URDF and meshes
- NAOqi Python bindings available in the runtime environment

## URDF Loading

This teleoperator supports two ways to load the NAO model:

1. By robot description name, using `--teleop.urdf_name=...`
2. By explicit URDF file path, using `--teleop.urdf_path=/absolute/path/to/nao.urdf`

If both are provided, `urdf_path` takes precedence.

When loading from `urdf_path`, the teleoperator also resolves
`package://nao_description/...` and `package://nao_meshes/...` paths
automatically, as long as the URDF is the original file inside a standard
`nao_description/urdf/...` layout and the meshes are installed next to the
`nao_robot` checkout. That means you do not need to bulk-edit the URDF just to
rewrite mesh paths.

## Installing The Official NAO Model

SoftBank does not allow the NAO meshes to be redistributed directly, so you
must install them from the official installer and combine them with the URDFs
from `ros-naoqi/nao_robot`.

The teleoperator does not care *where* the model lives — it only needs the
`nao_meshes` and `nao_robot` checkouts to sit side by side so the
`package://` mesh references resolve. The example below uses a `NAO_DIR`
variable so you can install anywhere; pick any directory you have write access
to (e.g. `~/nao` or a `external/nao` folder inside your project).

```bash
# Choose where to install the NAO model.
export NAO_DIR="$HOME/nao"
mkdir -p "$NAO_DIR"
cd "$NAO_DIR"

# 1. Get the URDFs.
git clone https://github.com/ros-naoqi/nao_robot.git

# 2. Get the meshes (not redistributable, so installed from the official run file).
wget -O naomeshes-0.6.7-linux-x64-installer.run \
  https://github.com/ros-naoqi/nao_meshes_installer/raw/master/naomeshes-0.6.7-linux-x64-installer.run
chmod +x naomeshes-0.6.7-linux-x64-installer.run

./naomeshes-0.6.7-linux-x64-installer.run \
  --mode text \
  --prefix "$NAO_DIR/nao_meshes"
```

After installation, the expected layout (relative to `$NAO_DIR`) is:

```text
$NAO_DIR/
  nao_meshes/
    meshes/
    texture/
  nao_robot/
    nao_description/
      urdf/
        naoV50_generated_urdf/
          nao.urdf
```

The `nao_meshes` and `nao_robot` directories must remain siblings — the
teleoperator derives the mesh package roots from the URDF's location (see
[URDF Loading](#urdf-loading)).

For NAO V5, point the teleoperator at:

`$NAO_DIR/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf`

The commands in [Usage](#usage) reuse this `$NAO_DIR` variable so you do not
have to hardcode an absolute path.

## Usage

### Choosing an arm

This teleoperator controls exactly one arm, selected with `--teleop.arm`
(`left` or `right`). The arm choice drives every arm-dependent setting, so you
normally only need to set `--teleop.arm`:

| Setting | `--teleop.arm=left` | `--teleop.arm=right` |
| --- | --- | --- |
| `arm_joint_names` | `LShoulderPitch`, `LShoulderRoll`, `LElbowYaw`, `LElbowRoll`, `LWristYaw` | `RShoulderPitch`, `RShoulderRoll`, `RElbowYaw`, `RElbowRoll`, `RWristYaw` |
| `target_link_name` | `l_gripper` | `r_gripper` |
| `hand_joint_name` | `LHand` | `RHand` |

These are derived automatically from `--teleop.arm`; you only pass
`--teleop.target_link_name`, `--teleop.arm_joint_names`, or
`--teleop.hand_joint_name` if you need to override the defaults for a
non-standard URDF.

Make sure the robot uses the **same** arm as the teleoperator
(`--robot.arm` must match `--teleop.arm`), otherwise the robot will reject the
action keys (e.g. it expects `LShoulderPitch.pos` but receives
`RShoulderPitch.pos`).

### Right arm

```bash
lerobot-teleoperate \
  --robot.type=nao_qi \
  --robot.robot_ip=127.0.0.1 \
  --robot.arm=right \
  --robot.enable_camera=false \
  --teleop.type=nao_ik \
  --teleop.arm=right \
  --teleop.urdf_path="$NAO_DIR/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf"
```

### Left arm

```bash
lerobot-teleoperate \
  --robot.type=nao_qi \
  --robot.robot_ip=127.0.0.1 \
  --robot.arm=left \
  --robot.enable_camera=false \
  --teleop.type=nao_ik \
  --teleop.arm=left \
  --teleop.urdf_path="$NAO_DIR/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf"
```

Then open `http://localhost:8080` and drag the target gizmo to move the selected NAO arm.

## Notes

- The implementation is intentionally one-arm only.
- The exact joint names and target link must match your NAO URDF.
- If your URDF exposes more actuated joints than the arm, the teleoperator keeps them locked.
- A typical working path is `.../nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf`.
- In the official `naoV50_generated_urdf`, `RHand`/`LHand` are joint names, not link names. Use `r_gripper` (right) or `l_gripper` (left) as the IK target link. These are selected automatically from `--teleop.arm`.
