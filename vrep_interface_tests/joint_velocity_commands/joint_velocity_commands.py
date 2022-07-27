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


1) Open the CoppeliaSim scene joint_velocity_commands.ttt
from cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/

Note: Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)
'''

from dqrobotics import *
from dqrobotics.interfaces.vrep  import DQ_VrepInterface
from dqrobotics.robot_control import ControlObjective
from dqrobotics.robot_control import DQ_PseudoinverseController
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH
import time
import numpy as np
from math import pi
from numpy import linalg as LA


vi = DQ_VrepInterface()

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
    robot_DH_theta = np.array([0, 0, 0, 0, 0, 0, 0])
    robot_DH_d = np.array([0.333, 0, 3.16e-1, 0, 3.84e-1, 0, 1.07e-1])
    robot_DH_a = np.array([0, 0, 8.25e-2, -8.25e-2, 0, 8.8e-2, 0])
    robot_DH_alpha = np.array([-pi / 2, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2, 0])
    robot_DH_type = np.array([0, 0, 0, 0, 0, 0, 0])
    robot_DH_matrix = np.array([robot_DH_theta, robot_DH_d, robot_DH_a, robot_DH_alpha, robot_DH_type])
    franka = DQ_SerialManipulatorDH(robot_DH_matrix)
    robot_base = 1 + E_ * 0.5 * DQ([0, 0.0413, 0, 0])
    franka.set_base_frame(robot_base)
    franka.set_reference_frame(robot_base)

    jointnames = ("Franka_joint1", "Franka_joint2", "Franka_joint3", "Franka_joint4",
                   "Franka_joint5", "Franka_joint6", "Franka_joint7")

    controller = DQ_PseudoinverseController(franka)
    controller.set_gain(0.5)
    controller.set_damping(0.05)
    controller.set_control_objective(ControlObjective.Translation)
    controller.set_stability_threshold(0.00001)

    xdesired = 1 + E_*0.5*DQ([0, 0.2, 0.3, 0.3])
    vi.set_object_pose("DesiredFrame", xdesired)
    i = 0
    #for i in range(iterations):
    while not controller.system_reached_stable_region():
        q = vi.get_joint_positions(jointnames)
        vi.set_object_pose("ReferenceFrame", franka.fkm(q))
        u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()))
        print("error: ", LA.norm(controller.get_last_error_signal()))
        print("Iteration: ", i)
        print("Is stable?: ", controller.system_reached_stable_region())
        vi.set_joint_target_velocities(jointnames, u)
        print("q_dot: ", vi.get_joint_velocities(jointnames))
        i=i+1
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
        "cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/joint_velocity_commands.ttt is open.")
    vi.disconnect_all()