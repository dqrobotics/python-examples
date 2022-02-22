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


1) Open the CoppeliaSim joint_torque_commands_no_gravity.ttt
from cpp-examples/cmake/vrep_interface_tests/joint_torque_commands/

Note: Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)
'''
import math

from dqrobotics import *
from dqrobotics.interfaces.vrep  import DQ_VrepInterface
from dqrobotics.robot_control import ControlObjective
from dqrobotics.robot_control import DQ_PseudoinverseController
from dqrobotics.robot_modeling import DQ_SerialManipulatorMDH
import time
import numpy as np
from math import pi
from numpy import linalg as LA


vi = DQ_VrepInterface()
iterations = 10000
## Always use a try-catch in case the connection with V-REP is lost
## otherwise your clientid will be locked for future use
try:
    ## Connects to the localhost in port 19997 with timeout 100ms and 10 retries for each method call
    vi.connect(19997, 100, 10)
    vi.set_synchronous(True)

    ## Starts simulation in V-REP
    print("Starting V-REP simulation...")
    vi.start_simulation()
    time.sleep(0.1)

    # Robot definition
    robot_MDH_theta = np.array([0, 0, 0, 0, 0, 0, 0])
    robot_MDH_d = np.array([0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 0])
    robot_MDH_a = np.array([0, 0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2])
    robot_MDH_alpha = np.array([0, -pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2])
    robot_MDH_type = np.array([0, 0, 0, 0, 0, 0, 0])
    robot_MDH_matrix = np.array([robot_MDH_theta, robot_MDH_d, robot_MDH_a, robot_MDH_alpha, robot_MDH_type])
    franka = DQ_SerialManipulatorMDH(robot_MDH_matrix)
    robot_base = 1 + E_ * 0.5 * DQ([0, 0.0413, 0, 0])
    franka.set_base_frame(robot_base)
    franka.set_reference_frame(robot_base)
    robot_effector = 1 + E_ * 0.5 * k_ * 1.07e-1
    franka.set_effector(robot_effector)

    jointnames = ("Franka_joint1", "Franka_joint2", "Franka_joint3", "Franka_joint4",
                   "Franka_joint5", "Franka_joint6", "Franka_joint7")

    Kp = 0.04
    Kv = 3*math.sqrt(Kp)
    qd = np.array([-0.70, -0.10, 1.66, -2.34, 0.40, 1.26, 0.070])
    vi.set_object_pose("DesiredFrame", franka.fkm(qd))

    for i in range(iterations):
        q = vi.get_joint_positions(jointnames)
        qerror =  qd-q
        q_dot = vi.get_joint_velocities(jointnames)
        qerror_dot = -q_dot

        vi.set_object_pose("ReferenceFrame", franka.fkm(q))

        vec_torques = Kp*qerror + Kv*qerror_dot
        vi.set_joint_torques(jointnames, vec_torques)
        vi.trigger_next_simulation_step()
        vec_torques_read = vi.get_joint_torques(jointnames)
        print("---------------------------------")
        print("Torques ref: ")
        print(vec_torques)
        print("Torques read: ")
        print(vec_torques_read)
        print("Applying torques...", iterations-i)
        print("Error...", LA.norm(qerror))

        vi.trigger_next_simulation_step()

    ## Stops simulation in V-REP
    print("Stopping V-REP simulation...")
    vi.stop_simulation()

    ## Disconnects V-REP
    vi.disconnect()

except Exception as exp:
    print(exp)
    print(
        "There was an error connecting to CoppeliaSim, please check that "
        "cpp-examples/cmake/vrep_interface_tests/joint_torque_commands/joint_torque_commands_no_gravity.ttt is open.")
    vi.disconnect_all()