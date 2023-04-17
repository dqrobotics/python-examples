from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.interfaces.vrep.robots import FrankaEmikaPandaVrepRobot
from dqrobotics.utils import *
from dqrobotics.utils.DQ_LinearAlgebra import pinv

import math
import time
import numpy as np

print('###############################################################')
print('In order for this example to run correctly, you have to')
print('open a scene in V-REP and drag a drop a "FrankaEmikaPanda.ttm" robot')
print('and remove or disable its child script')
print('before running this example.')
print('###############################################################')


## Creates a VrepInterface object
vi = DQ_VrepInterface()

## Always use a try-catch in case the connection with V-REP is lost
## otherwise your clientid will be locked for future use 
try:
    ## Connects to the localhost in port 19997 with timeout 100ms and 10 retries for each method call
    if not vi.connect(19997,100,10):
    	raise RuntimeError("Unable to connect to CoppeliaSim.")

    ## Starts simulation in V-REP
    print("Starting CoppeliaSim simulation...")
    vi.start_simulation()

    ## Instantiate the panda robot and get kinematic model
    robot_vrep = FrankaEmikaPandaVrepRobot("Franka",vi)
    robot = robot_vrep.kinematics()
    
    # Define a target pose
    xd    = robot.fkm((0,math.pi/2,0,math.pi/2,0,math.pi/2,0))

    ## Define the initial error as something big
    e = 1
    print("Starting control loop...")
    while np.linalg.norm(e)>0.01:
        theta = robot_vrep.get_q_from_vrep()
        x     = robot.fkm(theta)
        e     = vec8(x-xd)
        J     = robot.pose_jacobian(theta)
        
        u     = -0.1*np.matmul(pinv(J),e);
        theta = theta+u
        robot_vrep.send_q_target_to_vrep(theta)
        time.sleep(0.01)
    print("Control finished...")
        
    ## Stops simulation in V-REP
    print("Stopping V-REP simulation...")
    vi.stop_simulation()

    ## Disconnects V-REP
    vi.disconnect()

except KeyboardInterrupt:
    print("Interrupted by user")
    vi.stop_simulation()
    vi.disconnect()
except Exception as exp:
    print(exp)
    print("There was an error connecting to V-REP, please check that it is open and that the Kuka Robot is in the scene.")
    vi.disconnect_all()

