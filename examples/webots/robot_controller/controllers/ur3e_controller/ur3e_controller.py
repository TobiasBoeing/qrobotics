from controller import Robot as WbRobot
from spatialmath import SE3

from qrobotics.webots import WebotsUR3eRobotController


def main() -> None:
    wb = WbRobot()
    ur3e_controller = WebotsUR3eRobotController(wb)

    timestep_ms = int(wb.getBasicTimeStep())

    # MoveJ example
    q_home = ur3e_controller.HOME
    ur3e_controller.moveJ(q_home, duration=2.0)

    # MoveL example
    robot_model = ur3e_controller.get_robot_model()
    T0 = robot_model.fkine(q_home)
    T_target_1 = SE3(-0.2, 0.1, -0.2) * T0
    ur3e_controller.moveL(T_target_1, duration=3.0)

    T_target_2 = SE3(0.0, 0.0, 0.2) * T_target_1
    ur3e_controller.moveL(T_target_2, duration=3.0)

    # ---- Webots loop ----
    while wb.step(timestep_ms) != -1:
        ur3e_controller.step()


if __name__ == "__main__":
    main()
