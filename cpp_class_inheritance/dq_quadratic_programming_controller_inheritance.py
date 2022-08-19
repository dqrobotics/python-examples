from dqrobotics import *
from dqrobotics.robot_control import DQ_QuadraticProgrammingController, ControlObjective
from dqrobotics.robots import KukaLw4Robot
from dqrobotics.solvers import DQ_QuadprogSolver
import numpy as np
from math import pi


class DerivedControllerExample(DQ_QuadraticProgrammingController):
    def __init__(self, robot, solver):
        DQ_QuadraticProgrammingController.__init__(self, robot, solver)

    def compute_objective_function_symmetric_matrix(self, J, task_error):
        n = self._get_robot().get_dim_configuration_space()
        return J.T @ J + self.get_damping() * np.eye(n, n)

    def compute_objective_function_linear_component(self, J, task_error):
        return self.get_gain() * J.T @ task_error


solver = DQ_QuadprogSolver()
robot = KukaLw4Robot.kinematics()
q = [0, 0, 0, 0, 0, 0, 0]
xd = robot.fkm([pi / 2, 0, pi / 2, 0, -pi / 2, 0, pi / 2])

controller = DerivedControllerExample(robot, solver)
controller.set_control_objective(ControlObjective.Pose)
controller.set_gain(10)
controller.set_damping(0.001)

u = controller.compute_setpoint_control_signal(q, vec8(xd))

