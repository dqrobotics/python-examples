# dqrobotics-python-examples
Examples for dqrobotics-python

## Running examples

First, install dqrobotics-python3

```
python3 -m pip install dqrobotics
```

Each example can be run by executing it using python. 

For example, for `performance_evaluation/performance_evaluation.py` do
```
python3 performance_evaluation/performance_evaluation.py
```

An output similar to the one below indicates your example is running properly
```
***********************************************
Running dqrobotics-python performance test
LGPL3 dqrobotics developers               
Each function will be run:  10000  times.
The example robot has  7  DOFS.
***********************************************
np.random.rand(8,1)                [average s] 6.106615066528321e-07
DQ()                               [average s] 1.7670392990112305e-06
DQ+DQ                              [average s] 4.1569948196411134e-06
DQ*DQ                              [average s] 6.184101104736328e-06
DQ_kinematics.fkm                  [average s] 2.189640998840332e-05
DQ_kinematics.pose_jacobian        [average s] 4.8699569702148435e-05
DQ_kinematics.rotation_jacobian    [average s] 4.9999117851257324e-05
DQ_kinematics.translation_jacobian [average s] 7.204220294952393e-05
DQ_kinematics.line_jacobian        [average s] 8.921489715576171e-05
DQ_kinematics.plane_jacobian       [average s] 8.92798900604248e-05
***********************************************
```
