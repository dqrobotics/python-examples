import numpy as np

from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.interfaces.vrep.robots import LBR4pVrepRobot
from dqrobotics.robot_control import DQ_NumericalFilteredPseudoInverseController
from dqrobotics.robot_control import ControlObjective

cfg = {
    "verbose": False,
    "sampling_time": 0.001,
    "gain": 10,
    "isotropic_damping": 0.00001,
    "maximum_numerical_filtered_damping": 0.01,
    "singular_region_size": 0.01
}

if __name__ == "__main__":
    vi = DQ_VrepInterface()
    try:
        vi.connect(19997, 100, 100)
        vi.set_synchronous(True)
        vi.start_simulation()

        vrobot = LBR4pVrepRobot("LBR4p", vi)
        robot_kinematics = vrobot.kinematics()

        q = vrobot.get_q_from_vrep()
        xd = vi.get_object_pose("xd")

        controller = DQ_NumericalFilteredPseudoInverseController(robot_kinematics)
        controller.set_control_objective(ControlObjective.Pose)
        controller.set_gain(cfg["gain"])
        controller.set_damping(cfg["isotropic_damping"])
        # Parameteres specific to this controller
        controller.set_maximum_numerical_filtered_damping(cfg["maximum_numerical_filtered_damping"])
        controller.set_singular_region_size(cfg["singular_region_size"])

        T = cfg["sampling_time"]

        while not controller.system_reached_stable_region():
            u = controller.compute_setpoint_control_signal(q, vec8(xd))
            if cfg["verbose"]:
                print("Controller information: ")
                print(" Last error norm =", np.linalg.norm(controller.get_last_error_signal()))
                print(" Last Jacobian rank =", controller.get_last_jacobian_rank())

            q = q + u * T
            vrobot.send_q_target_to_vrep(q)
            vi.trigger_next_simulation_step()

    except Exception as e:
        print(e)
    except KeyboardInterrupt:
        print("Interrupted by user")

    vi.stop_simulation()
    vi.disconnect_all()
