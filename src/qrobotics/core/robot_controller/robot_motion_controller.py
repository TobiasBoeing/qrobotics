from __future__ import annotations

from collections import deque
from datetime import datetime

import numpy as np

from .commands import MoveJ, MoveL
from .robot_joint_controller import RobotJointController
from .robot_motion_planner import RobotMotionPlanner
from .trajectory import Trajectory


class RobotMotionController:
    def __init__(self, joint_controller: RobotJointController, planner: RobotMotionPlanner):
        self.joint_controller = joint_controller
        self.planner = planner
        self.queue: deque[object] = deque()
        self.active_trajectory: Trajectory | None = None

    def enqueue(self, cmd: object) -> None:
        self.queue.append(cmd)

    def step(self) -> None:
        """
        Call once per Webots timestep.
        """
        q_meas = self.joint_controller.get_actual_q()

        if self.active_trajectory is None:
            if not self.queue:
                return
            cmd = self.queue.popleft()
            planning_start_time = datetime.now()
            self.active_trajectory = self._plan(q_meas, cmd)
            planning_end_time = datetime.now()
            planning_duration = (planning_end_time - planning_start_time).total_seconds()
            print(
                f"Planned {type(cmd).__name__} in {planning_duration:.4f} s, trajectory length: {self.active_trajectory.N} steps"
            )

        q_des = self.active_trajectory.sample()
        self.joint_controller.set_target_q(q_des)

        if self.active_trajectory.done():
            self.active_trajectory = None

    def _plan(self, q0: np.ndarray, cmd: object) -> Trajectory:
        if isinstance(cmd, MoveJ):
            return self.planner.plan_moveJ(q0, cmd)
        if isinstance(cmd, MoveL):
            return self.planner.plan_moveL(q0, cmd)
        raise TypeError(f"Unknown command type: {type(cmd)}")
