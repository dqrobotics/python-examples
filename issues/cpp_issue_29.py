# A minimal example of cpp#29
# https://github.com/dqrobotics/cpp/issues/29
from math import pi

import numpy

from dqrobotics import *
from dqrobotics.robots import KukaLw4Robot
from dqrobotics.robot_control import ControlObjective, DQ_PseudoinverseController


# Get Kinematics
robot = KukaLw4Robot.kinematics()

# Initialize Controller
controller = DQ_PseudoinverseController(robot)
controller.set_control_objective(ControlObjective.Plane)
controller.set_primitive_to_effector(k_)
controller.set_gain(10.0)
controller.set_damping(0.001)

# Arbitrary joint values and task reference
q = numpy.array([0., pi/2.0, 0., pi/2.0, pi/10.0, pi/8.0, 0.])

# Print
print("Controller task variable = {}".format(controller.get_task_variable(q)))

# Test
controller.compute_setpoint_control_signal(q, vec8(DQ([1])))


