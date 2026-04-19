from __future__ import annotations

import numpy as np
import roboticstoolbox as rtb
from controller import Robot as WbRobot  # Webots API
from spatialmath import SE3

from qrobotics.core.robot_controller import RobotController

from .webots_robot_joint_controller import WebotsRobotJointController


class WebotsUR3eRobotController(RobotController):
    def __init__(self, wb_robot: WbRobot):
        self.HOME = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0], dtype=float)

        dh_params = [
            [0.00000, 0.15185, np.pi / 2, 0],  # joint 1 Base
            [-0.24355, 0.00000, 0.00000, 0],  # joint 2 Shoulder
            [-0.21320, 0.00000, 0.00000, 0],  # joint 3 Elbow
            [0.00000, 0.13105, np.pi / 2, 0],  # joint 4 Wrist1
            [0.00000, 0.08535, -np.pi / 2, 0],  # joint 5 Wrist 2
            [0.00000, 0.09210, 0, 0],  # joint 6 Wrist 3
        ]

        links = [rtb.RevoluteDH(d=link[1], a=link[0], alpha=link[2]) for link in dh_params]

        ur3e_robot = rtb.DHRobot(links, name="UR3e")
        ur3e_robot.base = SE3.Rz(np.pi)  # Webots uses different base frame

        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        joint_controller = WebotsRobotJointController(wb_robot, joint_names)

        super().__init__(ur3e_robot, joint_controller)
