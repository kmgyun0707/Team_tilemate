# cobot1/scraper_task_module.py
from __future__ import annotations

from typing import Optional

from cobot1.scraper_task import RobotConfig, GripperController, ScraperTask

_scraper: Optional[ScraperTask] = None

def initialize(node, robot_id, robot_model, tool, tcp, vel, acc):
    """
    DI 주입: node/설정으로 ScraperTask를 1회 생성해서 모듈 내부에 보관.
    """
    global _scraper
    cfg = RobotConfig(
        robot_id=robot_id,
        robot_model=robot_model,
        tool=tool,
        tcp=tcp,
        vel=vel,
        acc=acc,
    )
    gripper = GripperController(node)
    _scraper = ScraperTask(node=node, gripper=gripper, cfg=cfg)

def run_scraper_task():
    if _scraper is None:
        raise RuntimeError("scraper_task_module not initialized. Call initialize(...) first.")
    _scraper.run_once()
