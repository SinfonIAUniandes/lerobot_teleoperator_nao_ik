from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

# Arm-dependent defaults, keyed by `arm`. These must match the joint names the
# downstream robot (e.g. Pepper) expects for the corresponding arm.
ARM_JOINT_NAMES = {
    "left": ("LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"),
    "right": ("RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"),
}
ARM_TARGET_LINK_NAME = {"left": "l_gripper", "right": "r_gripper"}
ARM_HAND_JOINT_NAME = {"left": "LHand", "right": "RHand"}


@TeleoperatorConfig.register_subclass("nao_ik")
@dataclass
class NaoIkTeleopConfig(TeleoperatorConfig):
    arm: str = "right"
    urdf_name: str = ""
    urdf_path: str = ""
    # Arm-dependent fields. Leave as None to derive them from `arm`; set
    # explicitly to override.
    target_link_name: str | None = None
    hand_joint_name: str | None = None
    viser_port: int = 8080

    robot_ip: str = "127.0.0.1"
    robot_port: int = 9559
    app_name: str = "lerobot_nao_ik"

    disable_autonomous_life: bool = True
    use_startup_posture: bool = True
    startup_posture: str = "StandInit"
    startup_posture_speed: float = 0.2
    startup_settle_time_s: float = 1.5
    stiffness: float = 1.0
    speed_fraction: float = 0.15
    connect_timeout_s: float = 10.0

    initial_target_position: tuple[float, float, float] = (0.18, -0.15, 0.35)
    initial_target_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    arm_joint_names: tuple[str, ...] | None = None

    def __post_init__(self) -> None:
        arm = self.arm.lower()
        if arm not in ARM_JOINT_NAMES:
            raise ValueError(f"Unsupported arm '{self.arm}'. Expected 'left' or 'right'.")
        # Resolve arm-dependent fields from `arm` unless explicitly overridden.
        if self.arm_joint_names is None:
            self.arm_joint_names = ARM_JOINT_NAMES[arm]
        if self.target_link_name is None:
            self.target_link_name = ARM_TARGET_LINK_NAME[arm]
        if self.hand_joint_name is None:
            self.hand_joint_name = ARM_HAND_JOINT_NAME[arm]
