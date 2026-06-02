import threading
import time
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

from .config_nao_ik_teleop import NaoIkTeleopConfig
from .pyroki_snippets import solve_ik


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
        if self.config.arm.lower() not in {"left", "right"}:
            raise ValueError("arm must be 'left' or 'right'.")
        if not self.config.urdf_path and not self.config.urdf_name:
            raise ValueError("nao_ik requires urdf_path or urdf_name.")
        if self.config.speed_fraction <= 0.0 or self.config.speed_fraction > 1.0:
            raise ValueError("speed_fraction must be in the interval (0, 1].")
        if not 0.0 <= self.config.stiffness <= 1.0:
            raise ValueError("stiffness must be within [0, 1].")

    def _make_joint_mask(self) -> np.ndarray:
        selected = set(self.config.arm_joint_names)
        return np.array([
            1.0 if joint_name in selected else 0.0
            for joint_name in self.robot.joints.actuated_names
        ])

    def _solution_to_action(self, q_sol: np.ndarray) -> dict[str, float]:
        action = {}
        for joint_name in self.config.arm_joint_names:
            if joint_name in self.robot.joints.actuated_names:
                idx = self.robot.joints.actuated_names.index(joint_name)
                action[f"{joint_name}.pos"] = float(q_sol[idx])
        return action

    def _ik_worker(self):
        while self._is_connected:
            try:
                target_pos = np.array(self.ik_web_target.position, dtype=float)
                target_quat = np.array(self.ik_web_target.wxyz, dtype=float)
            except Exception:
                break

            q_sol = solve_ik(
                robot=self.robot,
                target_link_name=self.config.target_link_name,
                target_position=target_pos,
                target_wxyz=target_quat,
                joint_mask=self.joint_mask,
                prev_cfg=self._prev_cfg,
            )

            for idx, mask_value in enumerate(self.joint_mask):
                if mask_value == 0.0:
                    q_sol[idx] = self._prev_cfg[idx]

            self._prev_cfg = np.array(q_sol, copy=True)
            self.urdf_vis.update_cfg(self._prev_cfg)

            with self._lock:
                self._latest_q_sol = np.array(q_sol, copy=True)
                self._latest_action = self._solution_to_action(self._latest_q_sol)

            time.sleep(0.01)

    def connect(self) -> None:
        self.configure()
        self.urdf = self._load_urdf()
        self.robot = pk.Robot.from_urdf(self.urdf)
        self.joint_mask = self._make_joint_mask()
        self._prev_cfg = np.array(self.robot.joint_var_cls(0).default_factory(), copy=True)

        self.viser_server = viser.ViserServer(port=self.config.viser_port)
        self.viser_server.scene.add_grid("/ground", width=2, height=2)
        self.urdf_vis = ViserUrdf(self.viser_server, self.urdf, root_node_name="/nao")
        self.urdf_vis.update_cfg(self._prev_cfg)

        self.ik_web_target = self.viser_server.scene.add_transform_controls(
            "/ik_target",
            scale=0.1,
            position=self.config.initial_target_position,
            wxyz=self.config.initial_target_wxyz,
        )

        solve_ik(
            robot=self.robot,
            target_link_name=self.config.target_link_name,
            target_position=np.array(self.config.initial_target_position, dtype=float),
            target_wxyz=np.array(self.config.initial_target_wxyz, dtype=float),
            joint_mask=self.joint_mask,
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
