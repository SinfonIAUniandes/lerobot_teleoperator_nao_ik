import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pyroki as pk
import viser
from lerobot.teleoperators.teleoperator import Teleoperator
from robot_descriptions.loaders.yourdfpy import load_robot_description
from viser.extras import ViserUrdf
import yourdfpy
from yourdfpy import URDF

from .config_nao_ik_teleop import (
    ARM_INITIAL_TARGET_POSITION,
    ARM_JOINT_NAMES,
    ARM_TARGET_LINK_NAME,
    NaoIkTeleopConfig,
    sides_for_arm,
)
from .pyroki_snippets import solve_ik


@dataclass
class _ArmSpec:
    """Everything needed to drive a single arm's IK target."""

    side: str
    joint_names: tuple[str, ...]
    target_link_name: str
    joint_mask: np.ndarray
    gizmo: Any  # viser transform controls handle


class NaoIkTeleop(Teleoperator):
    config_class = NaoIkTeleopConfig
    name = "nao_ik"

    def __init__(self, config: NaoIkTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._ik_thread = None
        self._lock = threading.Lock()
        self._latest_q_sol = None
        self._latest_action = {}
        self._prev_cfg = None
        self.viser_server = None
        self.urdf_vis = None
        self._arm_specs: list[_ArmSpec] = []
        self._sides = sides_for_arm(config.arm)

    def _load_urdf(self):
        if self.config.urdf_path:
            urdf_path = Path(self.config.urdf_path).expanduser().resolve()
            nao_description_root = urdf_path.parents[2]
            package_roots = {
                "nao_description": nao_description_root,
                "nao_meshes": nao_description_root.parent.parent / "nao_meshes",
            }

            def filename_handler(fname: str) -> str:
                if fname.startswith("package://"):
                    package_name, _, rel_path = fname.removeprefix("package://").partition("/")
                    package_root = package_roots.get(package_name)
                    if package_root is not None:
                        return str((package_root / rel_path).resolve())
                return yourdfpy.filename_handler_magic(fname, dir=urdf_path.parent)

            urdf = URDF.load(urdf_path, filename_handler=filename_handler)
            self._ensure_joint_velocity_limits(urdf)
            return urdf
        if self.config.urdf_name:
            urdf = load_robot_description(self.config.urdf_name)
            self._ensure_joint_velocity_limits(urdf)
            return urdf
        raise ValueError("Either urdf_path or urdf_name must be configured for nao_ik.")

    @staticmethod
    def _ensure_joint_velocity_limits(urdf: URDF, default_velocity: float = np.pi) -> None:
        for joint in urdf.joint_map.values():
            if joint.type in {"fixed", "floating", "planar"}:
                continue
            if joint.limit is None:
                joint.limit = yourdfpy.urdf.Limit(
                    effort=0.0,
                    velocity=default_velocity,
                    lower=None,
                    upper=None,
                )
                continue
            if joint.limit.velocity is None:
                joint.limit.velocity = default_velocity

    def configure(self) -> None:
        if self.config.arm.lower() not in {"left", "right", "both"}:
            raise ValueError("arm must be 'left', 'right' or 'both'.")
        if not self.config.urdf_path and not self.config.urdf_name:
            raise ValueError("nao_ik requires urdf_path or urdf_name.")
        if self.config.speed_fraction <= 0.0 or self.config.speed_fraction > 1.0:
            raise ValueError("speed_fraction must be in the interval (0, 1].")
        if not 0.0 <= self.config.stiffness <= 1.0:
            raise ValueError("stiffness must be within [0, 1].")

    def _make_joint_mask(self, joint_names: tuple[str, ...]) -> np.ndarray:
        selected = set(joint_names)
        return np.array([
            1.0 if joint_name in selected else 0.0
            for joint_name in self.robot.joints.actuated_names
        ])

    def _target_link_for(self, side: str) -> str:
        # For a single arm, honour an explicit target_link_name override; for
        # 'both' the per-side default is always used.
        if len(self._sides) == 1 and self.config.target_link_name:
            return self.config.target_link_name
        return ARM_TARGET_LINK_NAME[side]

    def _initial_position_for(self, side: str) -> tuple[float, float, float]:
        # For a single arm, honour the configured initial position; for 'both'
        # use the per-side defaults so the two gizmos don't overlap.
        if len(self._sides) == 1:
            return self.config.initial_target_position
        return ARM_INITIAL_TARGET_POSITION[side]

    def _solution_to_action(self, q_sol: np.ndarray) -> dict[str, float]:
        action = {}
        for joint_name in self.config.arm_joint_names:
            if joint_name in self.robot.joints.actuated_names:
                idx = self.robot.joints.actuated_names.index(joint_name)
                action[f"{joint_name}.pos"] = float(q_sol[idx])
        return action

    def _ik_worker(self):
        while self._is_connected:
            cfg = np.array(self._prev_cfg, copy=True)
            for spec in self._arm_specs:
                try:
                    target_pos = np.array(spec.gizmo.position, dtype=float)
                    target_quat = np.array(spec.gizmo.wxyz, dtype=float)
                except Exception:
                    return

                q_sol = solve_ik(
                    robot=self.robot,
                    target_link_name=spec.target_link_name,
                    target_position=target_pos,
                    target_wxyz=target_quat,
                    joint_mask=spec.joint_mask,
                    prev_cfg=cfg,
                )

                # Keep joints outside this arm at their previous values so each
                # arm only moves its own joints.
                for idx, mask_value in enumerate(spec.joint_mask):
                    if mask_value == 0.0:
                        q_sol[idx] = cfg[idx]
                cfg = np.array(q_sol, copy=True)

            self._prev_cfg = cfg
            self.urdf_vis.update_cfg(self._prev_cfg)

            with self._lock:
                self._latest_q_sol = np.array(cfg, copy=True)
                self._latest_action = self._solution_to_action(self._latest_q_sol)

            time.sleep(0.01)

    def connect(self) -> None:
        self.configure()
        self.urdf = self._load_urdf()
        self.robot = pk.Robot.from_urdf(self.urdf)
        self._prev_cfg = np.array(self.robot.joint_var_cls(0).default_factory(), copy=True)

        self.viser_server = viser.ViserServer(port=self.config.viser_port)
        self.viser_server.scene.add_grid("/ground", width=2, height=2)
        self.urdf_vis = ViserUrdf(self.viser_server, self.urdf, root_node_name="/nao")
        self.urdf_vis.update_cfg(self._prev_cfg)

        # Build one IK target (gizmo + mask + link) per controlled arm.
        self._arm_specs = []
        for side in self._sides:
            joint_names = ARM_JOINT_NAMES[side]
            target_link_name = self._target_link_for(side)
            gizmo = self.viser_server.scene.add_transform_controls(
                f"/ik_target_{side}",
                scale=0.1,
                position=self._initial_position_for(side),
                wxyz=self.config.initial_target_wxyz,
            )
            self._arm_specs.append(
                _ArmSpec(
                    side=side,
                    joint_names=joint_names,
                    target_link_name=target_link_name,
                    joint_mask=self._make_joint_mask(joint_names),
                    gizmo=gizmo,
                )
            )

            # Warm up the JAX solver for this arm so the worker loop is responsive.
            solve_ik(
                robot=self.robot,
                target_link_name=target_link_name,
                target_position=np.array(self._initial_position_for(side), dtype=float),
                target_wxyz=np.array(self.config.initial_target_wxyz, dtype=float),
                joint_mask=self._make_joint_mask(joint_names),
                prev_cfg=self._prev_cfg,
            )

        self._is_connected = True
        self._ik_thread = threading.Thread(target=self._ik_worker, daemon=True)
        self._ik_thread.start()

    def disconnect(self) -> None:
        self._is_connected = False
        if self.viser_server:
            self.viser_server.stop()
        if self._ik_thread:
            self._ik_thread.join(timeout=1.0)

    def get_action(self) -> dict[str, float]:
        with self._lock:
            return self._latest_action.copy()

    @property
    def action_features(self) -> dict:
        return {f"{joint}.pos": float for joint in self.config.arm_joint_names}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass
