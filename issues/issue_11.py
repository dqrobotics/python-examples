from dqrobotics.robots import KukaLw4Robot
from dqrobotics.robot_modeling import DQ_SerialManipulator
import numpy as np

robot = KukaLw4Robot.kinematics()
q = np.array([0,0,0,0,0,0,0])
q_dot = np.array([0,0,0,0,0,0,0])

J = robot.pose_jacobian(q)
J_dot = robot.pose_jacobian_derivative(q, q_dot)

print("shapes")
print(J.shape)
print(J_dot.shape)
