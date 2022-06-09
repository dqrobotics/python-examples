from dqrobotics import *
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.utils import DQ_Geometry
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.interfaces.vrep.robots import LBR4pVrepRobot
from dqrobotics.robot_control import DQ_ClassicQPController, ControlObjective
from dqrobotics.solvers import DQ_QuadprogSolver

from line_segment_to_line_segment_example import show_line_segments_on_coppeliasim

import numpy as np

configuration = {
    "T": 0.001,  # Sampling time
    "d_safe": 0.01,  # Safe distance between shafts
    "vfi_gain": 10  # VFI gain, the larger the faster the approach towards obstacles
}


def adjust_relative_dq_from_coppeliasim(dq):
    if np.linalg.norm((dq - 1).vec8()) \
            > np.linalg.norm((dq + 1).vec8()):
        return -dq
    else:
        return dq


vi = DQ_VrepInterface()
try:
    vi.connect(19997, 100, 100)
    vi.set_synchronous(True)
    vi.start_simulation()

    # Initialize robots
    vrep_robot_0 = LBR4pVrepRobot("LBR4p#0", vi)
    robot_0_kinematics = vrep_robot_0.kinematics()
    robot_0_kinematics.set_effector(DQ([1.0]))  # Reset effector
    robot_0_q_init = vrep_robot_0.get_q_from_vrep()
    q_0 = robot_0_q_init

    vrep_robot_1 = LBR4pVrepRobot("LBR4p#1", vi)
    robot_1_kinematics = vrep_robot_1.kinematics()
    robot_1_kinematics.set_effector(DQ([1.0]))  # Reset effector
    robot_1_q_init = vrep_robot_1.get_q_from_vrep()
    q_1 = robot_0_q_init

    # Obtain end-effector transformations automagically from CoppeliaSim
    vi.set_object_pose("x1", robot_0_kinematics.fkm(robot_0_q_init))
    vi.set_object_pose("x2", robot_1_kinematics.fkm(robot_1_q_init))
    vi.trigger_next_simulation_step()
    # We might have issues here related to the DQ double cover (CoppeliaSim might return -x or x)
    robot_0_end_effector = adjust_relative_dq_from_coppeliasim(vi.get_object_pose("cs_x1", "x1"))
    robot_1_end_effector = adjust_relative_dq_from_coppeliasim(vi.get_object_pose("cs_x2", "x2"))
    robot_0_kinematics.set_effector(robot_0_end_effector)
    robot_1_kinematics.set_effector(robot_1_end_effector)

    # Define controller
    qp_solver = DQ_QuadprogSolver()
    robot_0_controller = DQ_ClassicQPController(robot_0_kinematics, qp_solver)
    robot_0_controller.set_control_objective(ControlObjective.Pose)
    robot_0_controller.set_gain(300.0)
    robot_0_controller.set_damping(0.01)

    # VFI
    D_safe = configuration["d_safe"] ** 2
    vfi_gain = configuration["vfi_gain"]

    while True:
        xd1 = vi.get_object_pose("xd1")

        robot_0_x = robot_0_kinematics.fkm(q_0)
        robot_0_t = translation(robot_0_x)
        robot_0_r = rotation(robot_0_x)
        robot_0_l = Ad(robot_0_r, k_)
        robot_0_m = cross(robot_0_t, robot_0_l)
        robot_0_line = robot_0_l + DQ.E * robot_0_m

        robot_0_p2_x = robot_0_x * conj(1 + 0.5 * DQ.E * k_ * translation(robot_0_end_effector).q[3])
        robot_0_p2_t = translation(robot_0_p2_x)

        robot_0_p1_Jx = robot_0_kinematics.pose_jacobian(q_0)
        robot_0_p1_Jt = DQ_Kinematics.translation_jacobian(robot_0_p1_Jx, robot_0_x)
        robot_0_Jl = DQ_Kinematics.line_jacobian(robot_0_p1_Jx, robot_0_x, k_)
        robot_0_p2_Jx = haminus8(conj(robot_0_end_effector)) @ robot_0_p1_Jx
        robot_0_p2_Jt = DQ_Kinematics.translation_jacobian(robot_0_p2_Jx, robot_0_p2_x)

        robot_1_x = robot_1_kinematics.fkm(q_1)
        robot_1_t = translation(robot_1_x)
        robot_1_r = rotation(robot_1_x)
        robot_1_l = Ad(robot_1_r, k_)
        robot_1_m = cross(robot_1_t, robot_1_l)
        robot_1_line = robot_1_l + DQ.E * robot_1_m

        robot_1_p2_x = robot_1_x * conj(1 + 0.5 * DQ.E * k_ * translation(robot_1_end_effector).q[3])
        robot_1_p2_t = translation(robot_1_p2_x)

        J_D = DQ_Kinematics.line_segment_to_line_segment_distance_jacobian(
            robot_0_Jl,
            robot_0_p1_Jt,
            robot_0_p2_Jt,
            robot_0_line,
            robot_0_t,
            robot_0_p2_t,
            robot_1_line,
            robot_1_p2_t,
            robot_1_p2_t
        )

        D = DQ_Geometry.line_segment_to_line_segment_squared_distance(
            robot_0_line,
            robot_0_t,
            robot_0_p2_t,
            robot_1_line,
            robot_1_p2_t,
            robot_1_p2_t
        )

        D_error = D - D_safe

        robot_0_controller.set_inequality_constraint(
            -J_D,
            np.column_stack((vfi_gain * D_error,))
        )

        robot_1_x = robot_0_kinematics.fkm(q_1)

        q_dot_0 = robot_0_controller.compute_setpoint_control_signal(q_0, vec8(xd1))
        q_0 = q_0 + q_dot_0 * configuration["T"]

        vrep_robot_0.send_q_target_to_vrep(q_0)
        vrep_robot_1.send_q_target_to_vrep(q_1)
        vi.set_object_pose("x1", robot_0_x)
        vi.set_object_pose("x2", robot_1_x)

        show_line_segments_on_coppeliasim(vi)

        vi.trigger_next_simulation_step()

except Exception as e:
    print(e)
except KeyboardInterrupt:
    print("Interrupted by user")

vi.stop_simulation()
vi.disconnect_all()
