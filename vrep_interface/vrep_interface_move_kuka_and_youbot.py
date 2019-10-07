import time
import numpy as np
from math import pi, sin, cos
import quadprog

from dqrobotics import *
from dqrobotics.utils import DQ_Geometry
from dqrobotics.robot_control import DQ_TaskSpacePseudoInverseController, DQ_ClassicQPController, ControlObjective
from dqrobotics.solvers import DQ_QuadraticProgrammingSolver, DQ_QuadprogSolver
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.interfaces.vrep.robots import LBR4pVrepRobot, YouBotVrepRobot


def get_plane_from_vrep(vrep_interface, plane_name, normal):
    plane_object_pose = vrep_interface.get_object_pose(plane_name)
    p = translation(plane_object_pose)
    r = rotation(plane_object_pose)
    n = Ad(r, normal)
    d = dot(p, n)
    return n + E_ * d


def get_line_from_vrep(vrep_interface, line_name, direction):
    line_object_pose = vrep_interface.get_object_pose(line_name)
    p = translation(line_object_pose)
    r = rotation(line_object_pose)
    l = Ad(r, direction)
    m = cross(p, l)
    return l + E_ * m


def compute_reference(simulation_parameters_inner, x0, t_inner):
    dispz = simulation_parameters_inner.dispz
    wd = simulation_parameters_inner.wd
    wn = simulation_parameters_inner.wn

    phi = (pi / 2.0) * sin(wn * t_inner)
    r = cos(phi / 2.0) + k_ * sin(phi / 2.0)

    z = dispz * cos(wd * t_inner) * k_
    p = 1 + E_ * 0.5 * z

    # Return pose
    xd = r * x0 * p

    # Return time derivative
    phidot = (pi / 2.0) * cos(wn * t_inner) * wn
    rdot = 0.5 * (-sin(phi / 2.0) + k_ * cos(phi / 2.0)) * phidot
    pdot = -E_ * 0.5 * dispz * wd * sin(wd * t_inner) * k_
    xd_dot = rdot * x0 * p + r * x0 * pdot

    return xd, xd_dot


def compute_constraints(youbot, plane_inner, cylinder1_inner, cylinder2_inner):
    robot_radius = 0.35
    radius_cylinder1 = 0.1
    radius_cylinder2 = 0.1

    youbot_base = youbot.get_chain_as_holonomic_base(0)
    youbot_base_pose = youbot_base.raw_fkm(youbot_q)
    youbot__base_Jx = youbot_base.raw_pose_jacobian(youbot_q, 2)

    t_inner = translation(youbot_base_pose)
    base_jt = youbot.translation_jacobian(youbot__base_Jx, youbot_base_pose)
    Jt = np.concatenate((base_jt, np.zeros((4, 5))), axis=1)

    j_dist_plane = youbot.point_to_plane_distance_jacobian(Jt, t_inner, plane_inner)
    dist_plane = DQ_Geometry.point_to_plane_distance(t_inner, plane_inner) - robot_radius

    j_dist_cylinder_1 = youbot.point_to_line_distance_jacobian(Jt, t_inner, cylinder1_inner)
    dist_cylinder1 = DQ_Geometry.point_to_line_squared_distance(t_inner, cylinder1_inner) - (radius_cylinder1 + robot_radius) ** 2

    j_dist_cylinder_2 = youbot.point_to_line_distance_jacobian(Jt, t_inner, cylinder2_inner)
    dist_cylinder2 = DQ_Geometry.point_to_line_squared_distance(t_inner, cylinder2_inner) - (radius_cylinder2 + robot_radius) ** 2

    j_constraint = np.concatenate((j_dist_plane, j_dist_cylinder_1, j_dist_cylinder_2), axis=0)
    b_constraint = np.array([dist_plane, dist_cylinder1, dist_cylinder2])

    return j_constraint, b_constraint


class SimulationParameters():
    def __init__(self, move_manipulator, wd, wn, total_time, dispz):
        self.move_manipulator = move_manipulator
        self.wd = wd
        self.wn = wn
        self.total_time = total_time
        self.dispz = dispz

# Creates a VrepInterface object
vi = DQ_VrepInterface()

# Always use a try-catch in case the connection with V-REP is lost
# otherwise your clientid will be locked for future use
try:
    # Connects to the localhost in port 19997 with timeout 100ms and 10 retries for each method call
    if not vi.connect(19997, 100, 5):
        raise Exception("Unable to connect to vrep!")

    simulation_parameters = SimulationParameters(
        move_manipulator=True,
        wd=0.5,
        wn=0.1,
        total_time=40.0,
        dispz=0.1)

    # Starts simulation in V-REP
    print("Starting V-REP simulation...")
    vi.start_simulation()

    # Initialize VREP robots
    lbr4p_vreprobot = LBR4pVrepRobot("LBR4p", vi)
    youbot_vreprobot = YouBotVrepRobot("youBot", vi)

    # Load DQ_robotics Kinematics
    lbr4p = lbr4p_vreprobot.kinematics()
    youbot = youbot_vreprobot.kinematics()

    # Initialize controllers
    lbr4p_controller = DQ_TaskSpacePseudoInverseController(lbr4p)
    lbr4p_controller.set_control_objective(ControlObjective.Pose)
    lbr4p_controller.set_gain(5)
    lbr4p_controller.set_damping(0.0)

    qp_solver = DQ_QuadprogSolver()
    youbot_controller = DQ_ClassicQPController(youbot, qp_solver)
    youbot_controller.set_control_objective(ControlObjective.Pose)
    youbot_controller.set_gain(5)
    youbot_controller.set_damping(0.01)

    sampling_time = 0.1
    tc = 0.0
    tcircle = 1 + E_ * 0.5 * 0.1 * j_
    rcircle = DQ(np.array([1.0]))

    # Get initial robot information
    lbr4p_q = np.array([0.0, 1.7453e-01, 0.0, 1.5708, 0.0, 2.6273e-01, 0.0])
    lbr4p_vreprobot.send_q_to_vrep(lbr4p_q)
    lbr4p_x0 = conj(lbr4p.reference_frame()) * lbr4p.fkm(lbr4p_q)
    youbot_q = youbot_vreprobot.get_q_from_vrep()

    print("Starting control loop...")
    first_iteration = True
    for t in np.arange(0.0, simulation_parameters.total_time, sampling_time):
        # Get Obstacles from VREP
        plane = get_plane_from_vrep(vi, "ObstaclePlane", k_)
        cylinder1 = get_line_from_vrep(vi, "ObstacleCylinder1", k_)
        cylinder2 = get_line_from_vrep(vi, "ObstacleCylinder2", k_)

        # Set LBR4p reference
        (lbr4p_xd_wrt_base, lbr4p_ff_wrt_base) = compute_reference(simulation_parameters, lbr4p_x0, t)
        lbr4p_xd = lbr4p.reference_frame() * lbr4p_xd_wrt_base
        lbr4p_ff = lbr4p.reference_frame() * lbr4p_ff_wrt_base

        # YouBot Trajectory
        youbot_xd = lbr4p_xd * (1 + 0.5 * E_ * 0.015 * k_) * j_
        youbot_ff = lbr4p_ff * (1 + 0.5 * E_ * 0.015 * k_) * j_

        # Modify the trajectory in order to draw a circle
        if first_iteration:
            first_iteration = False
        elif np.linalg.norm(youbot_controller.get_last_error_signal()) < 0.025:
            tc = tc + 0.01
            rcircle = cos(tc / 2) + k_ * sin(tc / 2)

        youbot_xd = youbot_xd * rcircle * tcircle
        youbot_ff = youbot_ff * rcircle * tcircle

        # Compute control signal for the arm
        lbr4p_u = lbr4p_controller.compute_tracking_control_signal(lbr4p_q, vec8(lbr4p_xd), vec8(lbr4p_ff))

        # Computer control signal for the youbot
        (Jconstraint, bconstraint) = compute_constraints(youbot, plane, cylinder1, cylinder2)
        youbot_controller.set_inequality_constraint(-Jconstraint, 1*bconstraint)
        youbot_u = youbot_controller.compute_tracking_control_signal(youbot_q, vec8(youbot_xd), vec8(youbot_ff))

        # Desired joint values
        lbr4p_q = lbr4p_q + lbr4p_u*sampling_time
        youbot_q = youbot_q + youbot_u*sampling_time

        # Send desired values
        lbr4p_vreprobot.send_q_to_vrep(lbr4p_q)
        youbot_vreprobot.send_q_to_vrep(youbot_q)

        time.sleep(sampling_time)
    print("Control finished...")

    # Stops simulation in V-REP
    print("Stopping V-REP simulation...")
    vi.stop_simulation()

    # Disconnects V-REP
    vi.disconnect()

except Exception as exp:
    print(exp)
    print(
        "There was an error connecting to V-REP, please check that it is open and that the Kuka Robot is in the scene.")
    vi.stop_simulation()
    vi.disconnect_all()
