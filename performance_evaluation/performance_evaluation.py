from dqrobotics import *
from dqrobotics.robots         import KukaLw4Robot
from dqrobotics.robot_modeling import DQ_Kinematics
import time
import numpy as np

RUN_COUNT = 10000;

robot = KukaLw4Robot.kinematics()

print("***********************************************")
print("Running dqrobotics-python performance test")
print("LGPL3 dqrobotics developers               ")
print("Each function will be run: ",RUN_COUNT," times.")
print("The example robot has ",robot.get_dim_configuration_space()," DOFS.")
print("***********************************************")


# Random vector creation
start = time.time()
for i in range(0,RUN_COUNT):
  dq_vec = np.random.rand(8,1)
end = time.time()
print("np.random.rand(8,1)                 [average s]",(((end-start))/RUN_COUNT))

# Dual quaternion from random vector creation
start = time.time()
for i in range(0,RUN_COUNT):
  a = DQ(np.random.rand(8,1))
end = time.time()
print("DQ()                                [average s]",(((end-start))/RUN_COUNT))

# Dual quaternion sum
start = time.time()
for i in range(0,RUN_COUNT):
  a = DQ(np.random.rand(8,1))
  b = DQ(np.random.rand(8,1))
  c = a+b
end = time.time()
print("DQ+DQ                               [average s]",(((end-start))/RUN_COUNT))

# Dual quaternion product
start = time.time()
for i in range(0,RUN_COUNT):
  a = DQ(np.random.rand(8,1))
  b = DQ(np.random.rand(8,1))
  c = a*b
end = time.time()
print("DQ*DQ                               [average s]",(((end-start))/RUN_COUNT))

# DQ_kinematics.fkm
start = time.time()
for i in range(0,RUN_COUNT):
  theta = np.random.rand(7,1)
  x     = robot.fkm(theta)
end = time.time()
print("DQ_SerialManipulator.fkm            [average s]",(((end-start))/RUN_COUNT))

# DQ_kinematics.pose_jacobian
start = time.time()
for i in range(0,RUN_COUNT):
  theta = np.random.rand(7,1)
  Jx    = robot.pose_jacobian(theta)
end = time.time()
print("DQ_SerialManipulator.pose_jacobian  [average s]",(((end-start))/RUN_COUNT))

# DQ_kinematics.rotation_jacobian
start = time.time()
for i in range(0,RUN_COUNT):
  theta = np.random.rand(7,1)
  Jx    = robot.pose_jacobian(theta)
  Jr    = DQ_Kinematics.rotation_jacobian(Jx)
end = time.time()
print("DQ_kinematics::rotation_jacobian    [average s]",(((end-start))/RUN_COUNT))

# DQ_kinematics.translation_jacobian
start = time.time()
for i in range(0,RUN_COUNT):
  theta = np.random.rand(7,1)
  Jx    = robot.pose_jacobian(theta)
  x     = robot.fkm(theta)
  Jt    = DQ_Kinematics.translation_jacobian(Jx,x)
end = time.time()
print("DQ_kinematics::translation_jacobian [average s]",(((end-start))/RUN_COUNT))

# DQ_kinematics.line_jacobian
start = time.time()
for i in range(0,RUN_COUNT):
  theta = np.random.rand(7,1)
  Jx    = robot.pose_jacobian(theta)
  x     = robot.fkm(theta)
  Jl    = DQ_Kinematics.line_jacobian(Jx,x,DQ.i)
end = time.time()
print("DQ_kinematics::line_jacobian        [average s]",(((end-start))/RUN_COUNT))

# DQ_kinematics.plane_jacobian
start = time.time()
for i in range(0,RUN_COUNT):
  theta = np.random.rand(7,1)
  Jx    = robot.pose_jacobian(theta)
  x     = robot.fkm(theta)
  Jpi   = DQ_Kinematics.plane_jacobian(Jx,x,DQ.i)
end = time.time()
print("DQ_kinematics::plane_jacobian       [average s]",(((end-start))/RUN_COUNT))

print("***********************************************")

