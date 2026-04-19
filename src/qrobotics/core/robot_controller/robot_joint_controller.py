from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class RobotJointController(ABC):
    @abstractmethod
    def get_actual_q(self) -> np.ndarray:
        pass

    @abstractmethod
    def set_target_q(self, target_q: np.ndarray) -> None:
        pass
