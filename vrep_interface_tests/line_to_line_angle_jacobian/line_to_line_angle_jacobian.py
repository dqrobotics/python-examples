'''
(C) Copyright 2022 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
Contributors:
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
Instructions:
Prerequisites:
- dqrobotics
- dqrobotics-vrep-interface
- quadprog

1) Open the CoppeliaSim scene test_dynamic_conic_constraint.ttt
from cpp-examples/cmake/vrep_interface_tests/line_to_line_angle_jacobian/
'''

from dqrobotics import *
from dqrobotics.interfaces.vrep  import DQ_VrepInterface
from dqrobotics.robot_control import ControlObjective
from dqrobotics.robot_control import DQ_ClassicQPController
from dqrobotics.robot_modeling import DQ_Kinematics, DQ_SerialManipulatorMDH
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.utils import DQ_Geometry
import time
import numpy as np
from math import pi, sin, cos


vi = DQ_VrepInterface()

USE_RESIDUAL = True  

## Always use a try-catch in case the connection with V-REP is lost
## otherwise your clientid will be locked for future use
try:
    ## Connects to the localhost in port 19997 with timeout 100ms and 10 retries for each method call
    vi.connect(19997, 100, 10)
    vi.set_synchronous(True)

    ## Starts simulation in CoppeliaSim
    print("Starting CoppeliaSim simulation...")
    vi.start_simulation()
    time.sleep(0.1)

    iterations = 15000
    jointnames = ("Franka_joint1", "Franka_joint2", "Franka_joint3", "Franka_joint4",
                   "Franka_joint5", "Franka_joint6", "Franka_joint7")

    #--------------------- Robot definition
    franka_mdh = np.array([[0,    0,     0,         0,      0,       0,      0],
                           [0.333, 0, 3.16e-1,       0, 3.84e-1,     0,      0],
                           [0,     0,     0,   8.25e-2, -8.25e-2,    0, 8.8e-2],
                           [0, -pi/2,   pi/2,     pi/2,    -pi/2,   pi/2, pi/2],
                           [0,     0,     0,         0,        0,     0,    0]])
    franka = DQ_SerialManipulatorMDH(franka_mdh)
    robot_base = 1 + E_ * 0.5 * DQ([0, 0.0413, 0, 0])
    franka.set_base_frame(robot_base)
    franka.set_reference_frame(robot_base)
    robot_effector = 1+E_*0.5*k_*1.07e-1
    franka.set_effector(robot_effector)

    # Update the base of the robot from CoppeliaSim
    new_base_robot = DQ([1])
    for i in range(10):
        new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_))

    franka.set_base_frame(new_base_robot)
    franka.set_reference_frame(new_base_robot)

    #------------------- Controller definition---------------------
    solver = DQ_QuadprogSolver()
    controller = DQ_ClassicQPController(franka, solver)
    controller.set_gain(0.5)
    controller.set_damping(0.1)
    controller.set_control_objective(ControlObjective.Translation)
    controller.set_stability_threshold(0.001)
    #---------------------------------------------------------------
    vfi_gain = 0.5
    safe_angle = 15*(pi/180)
    T = 0.005
    w = 0.2
    Alpha = 20*(pi/180)
    t = 0

    #for i in range(iterations):
    for t in np.arange(0.0, iterations*T, T):
        #t = i*T

        phi_t = Alpha*sin(w*t)
        phi_t_dot = Alpha * w * cos(w * t)
        r_dyn = cos(phi_t / 2) + i_ * sin(phi_t / 2)
        r_dyn_dot = (-sin(phi_t / 2) + i_ * cos(phi_t / 2)) * (phi_t_dot / 2)

        q = vi.get_joint_positions(jointnames)
        x = franka.fkm(q)
        J = franka.pose_jacobian(q)

        #  DQ workspace_pose is a unit dual quaternion that represent the position and orientation of a frame rigidly attached
        #  to the dynamic workspace line.
        workspace_line_pose = r_dyn + 0.5 * E_ * x.translation() * r_dyn

        #  Dynamic Workspace line
        workspace_attached_direction = k_
        workspace_line = r_dyn * workspace_attached_direction * (r_dyn.conj())
        vec_workspace_line_dot = (haminus4(k_ * r_dyn.conj()) + hamiplus4(r_dyn * k_) @ C4()) @ vec4(r_dyn_dot)

        # Robot Workspace line
        robot_attached_direction = -k_
        robot_line = (x.P()) * (robot_attached_direction) * (x.P().conj())

        # line-to-line-angle-Jacobian
        Jl = DQ_Kinematics.line_jacobian(J, x, robot_attached_direction)
        Jphi = DQ_Kinematics.line_to_line_angle_jacobian(
                        DQ_Kinematics.line_jacobian(J, x, robot_attached_direction),
                        robot_attached_direction,
                        workspace_line)
        residual_phi = DQ_Kinematics.line_to_line_angle_residual(robot_line,
                                                                 workspace_line,
                                                                 DQ(vec_workspace_line_dot))
        phi = DQ_Geometry.line_to_line_angle(robot_line, workspace_line)

        f = 2-2*cos(phi)
        fsafe = 2-2*cos(safe_angle)
        ferror = f-fsafe
        if -1*ferror < 0:
            print("-----RLINE_TO_LINE_ANGLE Constraint violated!!!!!!!!-------------------------")
        if not USE_RESIDUAL:
            residual_phi = 0
        print("residual_phi: ", residual_phi)
        b = np.array([vfi_gain*ferror + residual_phi])

        vi.set_object_pose("x", x)
        vi.set_object_pose("cone", workspace_line_pose)
        vi.set_joint_position("Revolute_joint_master", safe_angle)

        xdesired = x
        vi.set_object_pose("xd", xdesired)
        controller.set_inequality_constraint(Jphi, -b)
        u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()))
        print("Ending simulation at : ", iterations*T - t)

        vi.set_joint_target_velocities(jointnames, u)
        vi.trigger_next_simulation_step()

    ## Stops simulation in CoppeliaSim
    print("Stopping simulation...")
    vi.stop_simulation()

    ## Disconnects V-REP
    vi.disconnect()

except Exception as exp:
    print(exp)
    print(
        "There was an error connecting to CoppeliaSim, please check that "
        "cpp-examples/cmake/vrep_interface_tests/line_to_line_angle_jacobian/test_dynamic_conic_constraint.ttt is open.")
    vi.disconnect_all()