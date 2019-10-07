from dqrobotics import *
from dqrobotics.robots         import KukaLw4Robot
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.robot_modeling import DQ_CooperativeDualTaskSpace
from numpy import hstack, vstack
import numpy as np

theta = np.random.rand(7,1)

kuka1 = KukaLw4Robot.kinematics()
frame1 = 1 + DQ.E*0.5*DQ([0,-0.4,0,0])
kuka1.set_base_frame(frame1)
kuka1.set_reference_frame(frame1)

kuka2 = KukaLw4Robot.kinematics()
frame2 = 1+DQ.E*0.5*DQ([0, 0.4,0,0])
kuka2.set_base_frame(frame2)
kuka2.set_reference_frame(frame2)
two_arms = DQ_CooperativeDualTaskSpace(kuka1, kuka2)

q1_start = np.array([-1.6965, 2.2620, 1.5708, 1.4451, -0.4398, 0.0628, 0])
q2_start = np.array([1.6336, -0.8168, 1.5708, 1.5080, -0.2513, 0, 0])

q = hstack((q1_start, q2_start))

two_arms.absolute_pose_jacobian(q)
print(two_arms.absolute_pose_jacobian(q).shape)
print(two_arms.relative_pose_jacobian(q).shape)

