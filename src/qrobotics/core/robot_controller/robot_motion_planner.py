from __future__ import annotations

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from .commands import MoveJ, MoveL
from .trajectory import Trajectory


class RobotMotionPlanner:
    def __init__(self, robot: rtb.DHRobot, dt: float):
        self.robot = robot
        self.dt = dt

    def _compute_steps(self, duration: float | None, steps: int | None, min_steps: int = 2) -> int:
        if steps is not None:
            return max(int(steps), min_steps)
        if duration is not None:
            return max(int(np.ceil(duration / self.dt)), min_steps)
        # default: 2 seconds
        return max(int(np.ceil(2.0 / self.dt)), min_steps)

    def plan_moveJ(self, q0: np.ndarray, cmd: MoveJ) -> Trajectory:
        n = self._compute_steps(cmd.duration, cmd.steps)
        tj = rtb.jtraj(q0, cmd.q_target, n)
        return Trajectory(tj.q)

    def plan_moveL(self, q0: np.ndarray, cmd: MoveL) -> Trajectory:
        """
        Cartesian linear path (robust):
        - Choose number of samples based on a maximum Cartesian step (not only duration/dt)
        - Use robust IK: higher iteration limits + a few restarts around the seed
        - Optional: report which pose failed

        Notes:
        - Keeps orientation fixed because T_target is computed from T0 (same R).
        - If you want position-only IK (ignore orientation), see MASK option below.
        """

        # --- Tunables (start with these) ---
        MAX_CART_STEP = 0.01  # meters between consecutive poses (e.g. 1 cm). Use 0.002–0.01 for more robustness.
        MIN_STEPS = 20  # lower bound so short moves still have enough points
        IK_ILIMIT = 200  # LM iteration limit
        IK_SLIMIT = 100  # LM search limit (line search steps)
        IK_TOL = 1e-6
        RESTARTS = 6  # how many random restarts if seeded solve fails
        RESTART_STD = 0.2  # rad magnitude of random seed perturbation

        # Duration still matters: we will not generate fewer steps than duration/dt.
        n_time = self._compute_steps(cmd.duration, cmd.steps)

        T0 = self.robot.fkine(q0)  # SE3

        # --- compute steps based on Cartesian distance ---
        p0 = np.array(T0.t).reshape(3)
        p1 = np.array(cmd.T_target.t).reshape(3)
        dist = float(np.linalg.norm(p1 - p0))
        n_dist = int(np.ceil(dist / MAX_CART_STEP)) + 1  # +1 to include endpoints

        n = max(n_time, n_dist, MIN_STEPS)

        poses = rtb.ctraj(T0, cmd.T_target, n)

        q_list = []
        q_seed = q0.copy()

        for i, T in enumerate(poses):
            q_sol = self._ik_robust(
                T,
                q_seed,
                ilimit=IK_ILIMIT,
                slimit=IK_SLIMIT,
                tol=IK_TOL,
                restarts=RESTARTS,
                restart_std=RESTART_STD,
            )

            if q_sol is None:
                # Helpful diagnostics (position of the failing pose)
                p = np.array(T.t).reshape(3)
                raise RuntimeError(
                    f"IK failed during moveL at sample {i + 1}/{n}, "
                    f"pose position = [{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}], "
                    f"dist t sequence vertically (row wise).otal = {dist:.4f} m"
                )

            q_seed = q_sol
            q_list.append(q_sol)

        return Trajectory(np.vstack(q_list))

    def _ik(self, T: SE3, q_seed: np.ndarray) -> np.ndarray | None:
        """
        Seeded IK using robotics-toolbox.
        """
        # Common robust choice in RTB: ikine_LM
        sol = self.robot.ikine_LM(T, q0=q_seed)
        if not sol.success:
            return None
        return np.array(sol.q, dtype=float)

    def _ik_robust(
        self,
        T: SE3,
        q_seed: np.ndarray,
        *,
        ilimit: int = 200,
        slimit: int = 100,
        tol: float = 1e-6,
        restarts: int = 6,
        restart_std: float = 0.2,
    ) -> np.ndarray | None:
        """
        Robust IK wrapper around RTB ikine_LM:
        - try seeded solve first
        - if it fails, do a few randomized restarts around the seed
        """

        # 1) seeded attempt
        sol = self.robot.ikine_LM(T, q0=q_seed, ilimit=ilimit, slimit=slimit, tol=tol)

        if sol.success and np.isfinite(sol.q).all():
            return np.array(sol.q, dtype=float)

        # 2) random restarts near the seed
        dof = len(q_seed)
        for _ in range(restarts):
            q0 = q_seed + restart_std * (np.random.rand(dof) - 0.5)

            sol = self.robot.ikine_LM(T, q0=q0, ilimit=ilimit, slimit=slimit, tol=tol)

            if sol.success and np.isfinite(sol.q).all():
                return np.array(sol.q, dtype=float)

        return None
