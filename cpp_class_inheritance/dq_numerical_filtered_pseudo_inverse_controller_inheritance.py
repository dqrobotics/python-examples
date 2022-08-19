from dqrobotics import *
from dqrobotics.robot_control import DQ_NumericalFilteredPseudoinverseController, ControlObjective
from dqrobotics.robots import KukaLw4Robot
from dqrobotics.solvers import DQ_QuadprogSolver
import numpy as np
from math import pi


class DerivedControllerExample(DQ_NumericalFilteredPseudoinverseController):
    def __init__(self, robot):
        DQ_NumericalFilteredPseudoinverseController.__init__(self, robot)

    def compute_setpoint_control_signal(self, q, task_reference):
        # Below we simply use the Base classe's method, but instead you customize it here
        return DQ_NumericalFilteredPseudoinverseController.compute_setpoint_control_signal(self, q, task_reference)

    def compute_tracking_control_signal(self, q, task_reference, feed_forward):
        # Below we simply use the Base classe's method, but instead you customize it here
        return DQ_NumericalFilteredPseudoinverseController.compute_tracking_control_signal(self, q, task_reference, feed_forward)


solver = DQ_QuadprogSolver()
robot = KukaLw4Robot.kinematics()
q = [0, 0, 0, 0, 0, 0, 0]
xd = robot.fkm([pi / 2, 0, pi / 2, 0, -pi / 2, 0, pi / 2])

controller = DerivedControllerExample(robot)
controller.set_control_objective(ControlObjective.Pose)
controller.set_gain(10)
controller.set_damping(0.001)

u = controller.compute_setpoint_control_signal(q, vec8(xd))

