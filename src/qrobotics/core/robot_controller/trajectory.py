from __future__ import annotations

import numpy as np


class Trajectory:
    def __init__(self, q_samples: np.ndarray):
        """
        q_samples: shape (N, dof)
        """
        self.q = q_samples
        self.N = q_samples.shape[0]
        self.k = 0

    def sample(self) -> np.ndarray:
        qk = self.q[self.k]
        self.k = min(self.k + 1, self.N)  # clamp
        return qk

    def done(self) -> bool:
        if self.k >= self.N:
            return True
        return False
