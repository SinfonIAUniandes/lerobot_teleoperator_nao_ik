from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("nao_ik")
@dataclass
class NaoIkTeleopConfig(TeleoperatorConfig):
    arm: str = "right"
    urdf_name: str = ""
    urdf_path: str = ""
    target_link_name: str = "r_gripper"
    hand_joint_name: str = "RHand"
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

    arm_joint_names: tuple[str, ...] = (
        "RShoulderPitch",
        "RShoulderRoll",
        "RElbowYaw",
        "RElbowRoll",
        "RWristYaw",
    )
