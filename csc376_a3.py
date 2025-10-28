import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
from math import pi
import swift
import time

# Katherine Qin, Zehao Fan, Oct 2025

# Launch simulator swift
env = swift.Swift()
env.launch(realtime = True)

# Initialize two panda robots and their joint positions
panda1 = rtb.models.Panda()
panda2 = rtb.models.Panda()
panda1.q = panda1.qr
panda2.q = panda2.qr
# panda1.gripper[0].q

# Set robot position based on dimensions given in figure 3
panda1.base = sm.SE3(0.5,0,0) * sm.SE3.Rz(np.pi)
panda2.base = sm.SE3(-0.5,-0.1,0)

env.add(panda1)
env.add(panda2)

# initialize rod
# usage of sg.Cylinder() referenced https://www.gitlab.utcluj.ro/rcs/robotics-toolbox-python/-/blob/973f6e73b3e9bb39e59da1205cbf54e29035d2f9/roboticstoolbox/examples/fetch_vision.py
rod = sg.Cylinder(
    radius = 0.01,
    length = 0.3,
    base = sm.SE3(0,0,0)
)
rod.color = (0.8, 0, 0) # set rod to red color 
env.add(rod)

# callback funciton from the sliders in Swift which set 
# the joint angles of the robot to the value of the sliders
def set_joint(j, value):
    panda1.q[j] = np.deg2rad(float(value))

# Built on code from csc376_practical2_swift.py to implement slider for panda1
# Loop through each link in the robot and if it is a variable joint, add a slider to Swift to control it
j = 0
for link in panda1.links:
    if link.isjoint:

        # use a lambda as the callback function from Swift
        # j=j is used to set the value of j rather than the variable j
        # We use the HTML unicode format for the degree sign in the unit arg
        env.add(
            swift.Slider(
                lambda x, j=j: set_joint(j, x),
                min=np.round(np.rad2deg(link.qlim[0]), 2),
                max=np.round(np.rad2deg(link.qlim[1]), 2),
                step=1,
                value=np.round(np.rad2deg(panda1.q[j]), 2),
                desc="robot Joint " + str(int(str(j)) + 1),
                unit="&#176;",
            )
        )
        j += 1

while True:
    # Update the environment with the new robot pose
    env.step(0)

    # Update the rod position based on the end effector current position
    # Reference: https://petercorke.github.io/robotics-toolbox-python/IK/ik.html#inverse-kinematics
    # get current position of end effector 

    # update rod's position to pos of end effector 
    # T = panda1.fkine(panda1.q) * sm.SE3.Tx(10) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.45)

    time.sleep(0.01)    




