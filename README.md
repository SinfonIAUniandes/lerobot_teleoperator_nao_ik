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

Example setup inside this workspace:

```bash
cd /root/lerobot
mkdir -p external/nao
cd external/nao

wget -O naomeshes-0.6.7-linux-x64-installer.run \
  https://github.com/ros-naoqi/nao_meshes_installer/raw/master/naomeshes-0.6.7-linux-x64-installer.run
chmod +x naomeshes-0.6.7-linux-x64-installer.run

git clone https://github.com/ros-naoqi/nao_robot.git

./naomeshes-0.6.7-linux-x64-installer.run \
  --mode text \
  --prefix /root/lerobot/external/nao/nao_meshes
```

After installation, the expected layout is:

```text
/root/lerobot/external/nao/
  nao_meshes/
    meshes/
    texture/
  nao_robot/
    nao_description/
      urdf/
        naoV50_generated_urdf/
          nao.urdf
```

For NAO V5, point the teleoperator at:

`/root/lerobot/external/nao/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf`

## Usage

Example command:

```bash
lerobot-teleoperate \
  --robot.type=nao_qi \
  --robot.robot_ip=127.0.0.1 \
  --robot.arm=right \
  --robot.enable_camera=false \
  --teleop.type=nao_ik \
  --teleop.arm=right \
  --teleop.urdf_path=/root/lerobot/external/nao/nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf \
  --teleop.target_link_name=r_gripper
```

Then open `http://localhost:8080` and drag the target gizmo to move the selected NAO arm.

## Notes

- The implementation is intentionally one-arm only.
- The exact joint names and target link must match your NAO URDF.
- If your URDF exposes more actuated joints than the arm, the teleoperator keeps them locked.
- A typical working path is `.../nao_robot/nao_description/urdf/naoV50_generated_urdf/nao.urdf`.
- In the official `naoV50_generated_urdf`, `RHand` is a joint name, not a link name. Use `r_gripper` as the right-arm IK target link.
