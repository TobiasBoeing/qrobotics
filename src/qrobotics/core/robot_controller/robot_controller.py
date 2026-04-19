from __future__ import annotations

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from .commands import MoveJ, MoveL
from .robot_joint_controller import RobotJointController
from .robot_motion_controller import RobotMotionController
from .robot_motion_planner import RobotMotionPlanner


class RobotController:
    def __init__(self, robot_model: rtb.DHRobot, joint_controller: RobotJointController):
        self.robot_model = robot_model
        self.joint_controller = joint_controller

        self.motion_planner = RobotMotionPlanner(robot_model, dt=0.1)
        self.motion_controller = RobotMotionController(joint_controller, self.motion_planner)

    def moveL(self, target: SE3, duration: float) -> None:
        cmd = MoveL(T_target=target, duration=duration)
        self.motion_controller.enqueue(cmd)

    def moveJ(self, target: np.ndarray, duration: float) -> None:
        cmd = MoveJ(q_target=target, duration=duration)
        self.motion_controller.enqueue(cmd)

    def get_actual_q(self) -> np.ndarray:
        return self.joint_controller.get_actual_q()

    def get_actual_T(self) -> SE3:
        q = self.get_actual_q()
        T = self.robot_model.fkine(q)
        return T

    def get_robot_model(self) -> rtb.DHRobot:
        return self.robot_model

    def step(self) -> None:
        self.motion_controller.step()
