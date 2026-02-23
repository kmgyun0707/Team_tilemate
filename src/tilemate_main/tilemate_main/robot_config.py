# tilemate_main/robot_config.py
from dataclasses import dataclass

@dataclass
class RobotConfig:
    robot_id: str = "dsr01"
    robot_model: str = "m0609"
    tool: str = "Tool Weight"
    tcp: str = "GripperDA_v1"
    vel: float = 40
    acc: float = 60
