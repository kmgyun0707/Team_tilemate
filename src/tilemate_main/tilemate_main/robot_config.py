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
    
@dataclass
class GripperConfig:
    GRIPPER_NAME: str = "rg2"
    #real
    TOOLCHARGER_IP: str = "192.168.1.1"
    TOOLCHARGER_PORT: str = "502"
    #sim
    # TOOLCHARGER_IP: str = "0.0.0.0"
    # TOOLCHARGER_PORT: str = "1234"
