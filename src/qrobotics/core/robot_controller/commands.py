from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from spatialmath import SE3


@dataclass(frozen=True)
class MoveJ:
    q_target: np.ndarray
    steps: int | None = None  # if None, computed from duration/dt
    duration: float | None = None  # seconds (optional)


@dataclass(frozen=True)
class MoveL:
    T_target: SE3
    steps: int | None = None
    duration: float | None = None
