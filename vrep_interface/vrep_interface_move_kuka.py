from dqrobotics import *
from dqrobotics.interfaces       import VrepInterface
from dqrobotics.robots           import KukaLw4Robot
from dqrobotics.utils.DQ_LinearAlgebra import *
import math
import time
import numpy as np

print('###############################################################')
print('In order for this example to run correctly, you have to')
print('open a scene in V-REP and drag a drop a "LBR_iiwa_14_R820" robot')
print('and remove or disable its child script')
print('before running this example.')
print('###############################################################')


## Creates a VrepInterface object
vi = VrepInterface()

## Always use a try-catch in case the connection with V-REP is lost
## otherwise your clientid will be locked for future use 
try:
    ## Connects to the localhost in port 19997 with timeout 100ms and 10 retries for each method call
    vi.connect(19997,100,10)

    ## Starts simulation in V-REP
    print("Starting V-REP simulation...")
    vi.start_simulation()
    
    ## Store joint names
    joint_names = ("LBR_iiwa_14_R820_joint1","LBR_iiwa_14_R820_joint2","LBR_iiwa_14_R820_joint3","LBR_iiwa_14_R820_joint4","LBR_iiwa_14_R820_joint5","LBR_iiwa_14_R820_joint6","LBR_iiwa_14_R820_joint7")
    ## Get joint handles
    joint_handles = vi.get_object_handles(joint_names)
    
    ## Defining robot kinematic model
    robot = KukaLw4Robot.kinematics()
    xd    = robot.fkm((0,math.pi/2,0,math.pi/2,0,math.pi/2,0))
    
    ## Defining target joints
    theta        = vi.get_joint_positions(joint_handles,VrepInterface.OP_STREAMING)
    
    ## Define error as something big
    e = 1
    print("Starting control loop...")
    while np.linalg.norm(e)>0.01:
        theta = vi.get_joint_positions(joint_handles,VrepInterface.OP_BUFFER)
        x     = robot.fkm(theta)
        e     = vec8(x-xd)
        J     = robot.pose_jacobian(theta)
        
        u     = -0.01*np.matmul(pinv(J),e);
        theta = theta+u
        vi.set_joint_target_positions(joint_handles,theta,VrepInterface.OP_ONESHOT)
        time.sleep(0.01)
    print("Control finished...")
        
    ## Stops simulation in V-REP
    print("Stopping V-REP simulation...")
    vi.stop_simulation()

    ## Disconnects V-REP
    vi.disconnect()

except Exception as exp:
    print(exp)
    print("There was an error connecting to V-REP, please check that it is open and that the Kuka Robot is in the scene.")
    vi.disconnect_all()

