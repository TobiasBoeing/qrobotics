from __future__ import annotations

import numpy as np
from controller import Motor, PositionSensor
from controller import Robot as WbRobot

from qrobotics.core.robot_controller import RobotJointController


class WebotsRobotJointController(RobotJointController):
    def __init__(self, wb: WbRobot, joint_names: list[str]):
        self.wb = wb
        self.dt = int(self.wb.getBasicTimeStep()) / 1000.0
        self.timestep_ms = int(wb.getBasicTimeStep())

        self.motors = []
        self.sensors = []

        for name in joint_names:
            motor: Motor = self.wb.getDevice(name)
            sensor: PositionSensor = self.wb.getDevice(name + "_sensor")

            sensor.enable(int(self.wb.getBasicTimeStep()))
            self.motors.append(motor)
            self.sensors.append(sensor)

    def get_actual_q(self) -> np.ndarray:
        q = np.array([s.getValue() for s in self.sensors], dtype=float)
        return q

    def set_target_q(self, q_des: np.ndarray) -> None:
        for m, q in zip(self.motors, q_des, strict=False):
            m.setPosition(float(q))
