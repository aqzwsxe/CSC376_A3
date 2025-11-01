import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
from math import pi
import swift
import time

# Katherine Qin, Zehao Fan, Oct 2025
# References: Lecture slides, code from practicals, and other online resources below
# Inverse kinematics: https://petercorke.github.io/robotics-toolbox-python/IK/ik.html#inverse-kinematics
# sg.Cylinder: https://www.gitlab.utcluj.ro/rcs/robotics-toolbox-python/-/blob/973f6e73b3e9bb39e59da1205cbf54e29035d2f9/roboticstoolbox/examples/fetch_vision.py

# Launch simulator swift
env = swift.Swift()
env.launch(realtime = True)

# Initialize two panda robots and their joint positions
panda1 = rtb.models.Panda()
panda2 = rtb.models.Panda()
panda1.q = panda1.qr
panda2.q = panda2.qr
panda1.grippers[0].q = [0.01, 0.01]
panda2.grippers[0].q = [0.01, 0.01]
 
# Set robot position based on dimensions given in figure 3
panda1.base = sm.SE3(0.5,0,0) * sm.SE3.Rz(np.pi)
#panda2.base = sm.SE3(-0.5,-0.1,0)
panda2.base = sm.SE3(-0.5,0,0)

env.add(panda1)
env.add(panda2)

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

# initialize rod parameters
rod_radius = 0.01
rod_length = 0.3

ee_pose1 = panda1.fkine(panda1.q)
ee_pose2 = panda2.fkine(panda2.q)
        
#Create the transformation matrix for the rod
R = sm.SO3.Ry(-np.pi/2) 
rod_pose_panda1 = ee_pose1 * sm.SE3(0,0,0) * sm.SE3(R)

# update the rod position based on the current end effector position
rod = sg.Cylinder(
    radius = rod_radius, 
    length = rod_length, 
    pose = rod_pose_panda1
    )
rod.color = (0.8, 0, 0)
env.add(rod)

# compute the relative transformation of panda1's ee to rod_pose
# This is used to update the rod's new position relative to its original position 
# we use .inv() to get the inverse of the matrix ee_pose1
T_offset_panda1 = ee_pose1.inv() * rod_pose_panda1 

# Set grasp offset to indicate the pose of Panda2's grippers relative to the rod
GRASP_OFFSET = 0.215

previous_q = panda2.q.copy()

while True:

    # get current position of end effectors
    ee_pose1 = panda1.fkine(panda1.q)
    ee_pose2 = panda2.fkine(panda2.q)

    # apply new position for rod relative to ee of Panda1
    rod.T = ee_pose1 * T_offset_panda1 * sm.SE3(0, 0, -rod_length/4)

    # find offset from panda1's ee to center of rod
    T_p1_ee_to_rod_center = sm.SE3(rod_length/2, 0, 0)

    # find pose T_p2_target for the desired Panda2 end effector pose
    # we set Panda2's gripper to grip the very end of the rod and rotate the gripper 180 deg around Z to align with the rod
    T_rod_center_to_p2_ee_target = sm.SE3(-(rod_length/2 - GRASP_OFFSET), 0, 0) @ sm.SE3.Rz(np.pi)
    T_rod_W_new = ee_pose1 @ T_p1_ee_to_rod_center
    T_p2_target = T_rod_W_new @ T_rod_center_to_p2_ee_target
    
    # T_p2_target is in world frame. We need to convert it to local frame
    T_local = panda2.base.inv() * T_p2_target

    # compute inverse kinematics 
    IK_sol = panda2.ikine_LM(
            T_local,
            q0=previous_q,
            mask=[1, 1, 1, 1, 1, 1],
            ilimit=50,
            slimit=10,
        )
    
    # if IK_sol computes successfully, update joint angles of panda2
    if IK_sol.success:
        panda2.q = IK_sol.q
        previous_q = panda2.q.copy() 

    # update the environment with the new robot pose
    env.step(0)
    time.sleep(0.05)    




