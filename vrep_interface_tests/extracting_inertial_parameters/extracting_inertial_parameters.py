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


1) Open the CoppeliaSim scene extracting_inertial_parameters.ttt
from cpp-examples/cmake/vrep_interface_tests/extracting_inertial_parameters/

Note: Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)


'''

from dqrobotics import *
from dqrobotics.interfaces.vrep  import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH
import time
import numpy as np
from math import pi


vi = DQ_VrepInterface()


def get_mean_inertia_tensor(inertia_list):
    [m, n] = inertia_list.shape
    trials = int(n / 3)
    joints = int(m / 3)
    inertia_mean = np.zeros((joints*3, 3))
    aux = np.zeros(trials)
    for row in range(joints*3):
        for j in range(3):
            for i in range(trials):
                aux[i] = inertia_list[row, i*3+j]
            inertia_mean[row, j] = np.mean(aux)
    return inertia_mean



def get_mean_com(com_list):
    [m, n] = com_list.shape
    trials = int(n)
    joints = int(m/3)
    com_mean = np.zeros((joints*3, 1))
    aux = np.zeros(trials)
    for row in range(joints*3):
        for i in range(trials):
            aux[i] = com_list[row, i]
            com_mean[row, 0] = np.mean(aux)
    return com_mean


def get_rotation_matrix(r):
    vecr = r.vec4()
    w = vecr[0]
    a = vecr[1]
    b = vecr[2]
    c = vecr[3]
    R = np.array([[1-2*(b*b +c*c), 2*(a*b-w*c), 2*(a*c+w*b)],
                  [2*(a*b+w*c), 1-2*(a*a+c*c),  2*(b*c-w*a)],
                  [2*(a*c-w*b), 2*(b*c+w*a),   1-2*(a*a+b*b)]])
    return R



## Always use a try-catch in case the connection with V-REP is lost
## otherwise your clientid will be locked for future use
try:
    ## Connects to the localhost in port 19997 with timeout 100ms and 10 retries for each method call
    vi.connect(19997, 100, 10)
    vi.set_synchronous(True)

    ## Starts simulation in V-REP
    print("Starting V-REP simulation...")
    vi.start_simulation()
    time.sleep(0.2)

    jointnames = ("Franka_joint1", "Franka_joint2", "Franka_joint3", "Franka_joint4",
                   "Franka_joint5", "Franka_joint6", "Franka_joint7")

    linknames = ("Franka_link2_resp", "Franka_link3_resp", "Franka_link4_resp",
                  "Franka_link5_resp", "Franka_link6_resp", "Franka_link7_resp", "Franka_link8_resp")

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
    #------------------------------------------------------------------------------------
    # We extract the inertial parameters in four different robot configurations. After, we compute the mean of the readings.

    targetPos1 = np.array([0, 0, 0, -pi/2, 0, pi/2, 0])
    targetPos2 = np.array([90*pi/180, 90*pi/180, 135*pi/180, -45*pi/180, 90*pi/180, 180*pi/180, 0])
    targetPos3 = np.array([-90*pi/180, 90*pi/180, 135*pi/180, -45*pi/180, 90*pi/180, 180*pi/180, 0])
    targetPos4 = np.array([0, 0, 0, -90*pi/180, 0, 90*pi/180, 0])
    list_positions = np.vstack([targetPos1, targetPos2, targetPos3, targetPos4])
    number_of_positions = list_positions.shape[0]

    inertia_list = np.zeros((np.size(linknames)*3, number_of_positions*3))
    com_list = np.zeros((np.size(linknames)*3, number_of_positions))
    mass_list = np.zeros(7)
    print("Number of positions: ", number_of_positions)

    for k in range(number_of_positions):
        print("Setting position: ", k+1)
        for i in range(500):
            vi.set_joint_target_positions(jointnames, list_positions[k])
            vi.trigger_next_simulation_step()
            q = vi.get_joint_positions(jointnames)
            vi.set_object_pose("ReferenceFrame", franka.fkm(q))

        for j in range(np.size(linknames)):
            link = linknames[j]
            mass = vi.get_mass(link)
            mass_list[j] = mass
            q = vi.get_joint_positions(jointnames)
            xi = franka.fkm(q, j)
            vi.set_object_pose("ReferenceFrame", xi)
            R = get_rotation_matrix(xi.P())
            #compute the Inertia tensor of each link expressed in the absolute frame.
            I_absolute_frame = vi.get_inertia_matrix(link, DQ_VrepInterface.ABSOLUTE_FRAME)

            #compute the Inertia tensor of each link expressed in the Denavit Hartenberg frames.
            I_DH_frame = R.T @ I_absolute_frame @ R
            inertia_list[j*3:(3+j*3), k*3:(3+k*3)] = I_DH_frame

            #compute the center of mass of each link expressed in the absolute frame.
            com0 = vi.get_center_of_mass(link, DQ_VrepInterface.ABSOLUTE_FRAME)
            xcom0 = 1 + E_ * 0.5 * DQ([0, com0[0], com0[1], com0[2]])

            #We compute the constant rigid transformation of the center of mass with respect to the DH frames.
            vecxc = vec3(translation(xi.conj() * xcom0))
            com_list[j*3:(3+j*3), k] = vecxc.reshape(3,)
            vi.trigger_next_simulation_step()

    for j in range(np.size(linknames)):
        print("-------------------------")
        print("Link: "+linknames[j]+". ")
        print("Center of mass: ")
        print((get_mean_com(com_list)[j*3:(3+j*3)]).T)
        print("Inertia tensor: ")
        print(get_mean_inertia_tensor(inertia_list)[j*3:(3+j*3), :])
        print("Mass: ")
        print(mass_list[j])




    ## Stops simulation in V-REP
    print("Stopping V-REP simulation...")
    vi.stop_simulation()

    ## Disconnects V-REP
    vi.disconnect()

except Exception as exp:
    print(exp)
    print(
        "There was an error connecting to VREP, please check that "
        "cpp-examples/cmake/vrep_interface_tests/extracting_inertial_parameters/extracting_inertial_parameters.ttt is open.")
    vi.disconnect_all()