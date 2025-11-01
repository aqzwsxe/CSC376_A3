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
panda1.grippers[0].q = [0.01, 0.01]
panda2.grippers[0].q = [0.01, 0.01]

# Set robot position based on dimensions given in figure 3
panda1.base = sm.SE3(0.5,0,0) * sm.SE3.Rz(np.pi)
panda2.base = sm.SE3(-0.5,-0.1,0)

env.add(panda1)
env.add(panda2)



# initialize rod parameters
rod_radius = 0.01
# Calculate initial end-effector poses
ee_pose1 = panda1.fkine(panda1.q)
#Create the transformation matrix for the rod
R = sm.SO3.Ry(-np.pi/2)
rod_pose_panda1 = ee_pose1 * sm.SE3(0,0,0) * sm.SE3(R)

ee_pose2 = panda2.fkine(panda2.q)

# Define the position: the midpoint between the two end-effector positions
p1_t = ee_pose1.t
p2_t = ee_pose2.t
rod_length = 0.4
rod_center_pos = (p1_t + p2_t) / 2

# Define the Orientation
v_p1_to_p2 = p2_t - p1_t
v_p2_to_p1 = p1_t - p2_t

R_rod_world = sm.SE3.OA([0, 0, 1], v_p2_to_p1).R

# Rotation along the z axis
R_rod = sm.SE3.OA([0, 0, 1], v_p2_to_p1).R
R_rod = sm.SO3(R_rod)  # wrap 3x3 rotation in SO3

# Rotation along the z axis
v_p2_to_p1 = p1_t - p2_t

v = p2_t - p1_t
v = v / np.linalg.norm(v)

# The world axis
world_up = np.array([0, 0, 1])
y_axis = np.cross(world_up, v)
y_axis /= np.linalg.norm(y_axis)
# Compute Z-axis as perpendicular to X (rod) and Y
z_axis = np.cross(v, y_axis)

# 1. Construct rotation matrix (R_initial): columns are X, Y, Z axes of target frame
# Length is along X-axis (v)
R_initial = sm.SO3(np.column_stack((v, y_axis, z_axis)))

# 2. Final Alignment: Rotate -90 degrees about Y-axis.
# This maps the Cylinder's internal Z-axis (length) onto the frame's X-axis.
R_align = sm.SO3.Ry(-np.pi/2)

# 3. Combine: R_final = R_initial * R_align
R_final = R_initial @ R_align

# Construct SE3 pose of rod
T_rod_W_init = sm.SE3.Rt(R_final.R, rod_center_pos)
rod = sg.Cylinder(
    radius=rod_radius,
    length=rod_length,
    pose=T_rod_W_init.A
)


rod.color = (0.8, 0, 0)
env.add(rod)

# Relative to Panda 1's End-Effector
GRASP_OFFSET = 0.10
# T_p1_ee_to_rod_center is the offset from P1_EE (origin) to ROD_CENTER
# It must account for the gripper's depth AND half the rod's length.
T_p1_ee_to_rod_center = sm.SE3(rod_length/2 + GRASP_OFFSET, 0, 0)
# Relative to Panda 2's End-Effector
T_p2_ee_to_rod_center = ee_pose2.inv() * T_rod_W_init

# The Rod center to Panda 2's End-Effector
R_flip = sm.SE3.Rz(np.pi)
# P2's EE (flange) must be positioned L/2 away from the center, plus the GRASP_OFFSET
# along the positive local Z-axis (which is where P2 grips).
T_rod_center_to_p2_ee_target = sm.SE3(-(rod_length/2 + GRASP_OFFSET), 0, 0) @ sm.SE3.Rz(np.pi)
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




# compute the relative transformation of panda1's ee to rod_pose
# This is used to update the rod's new position relative to its original position
# we use .inv() to get the inverse of the matrix ee_pose1
T_offset_panda1 = ee_pose1.inv() @ T_rod_W_init

# Same idea of the relative transformation but for panda2
# rod_pose_panda2 = rod.T * sm.SE3(0, 0, rod_length/2)
# T_offset_panda2 = ee_pose2.inv() * rod_pose_panda2


previous_q2 = panda2.q.copy()
# Define a simple trajectory for Panda 1's first joint (Joint 1)

# Define the automated path for Panda 1 (Driver)
q1_path = np.linspace(panda1.qr[0], 1.5, 50)
q1_path = np.concatenate((q1_path, np.linspace(1.5, -1.5, 100)))
q1_path = np.concatenate((q1_path, np.linspace(-1.5, panda1.qr[0], 50)))

# --- MAIN EXECUTION LOOP ---
try:
    loop_count = 0

    # 1. RUN AUTOMATED PATH TEST
    while loop_count < len(q1_path):
        panda1.q[0] = q1_path[loop_count]

        # 1) Update panda1 EE (Driver)
        T_p1_ee_new = panda1.fkine(panda1.q)

        # 2) Calculate new rod transform based on fixed offset from Panda 1
        T_rod_W_new = T_p1_ee_new @ T_p1_ee_to_rod_center
        T_p2_target = T_rod_W_new @ T_rod_center_to_p2_ee_target

        # 3) Desired Panda2 EE target: APPLY THE FIXED KINEMATIC OFFSET
        T_p2_target = T_rod_W_new @ T_rod_center_to_p2_ee_target  # <--- FIX: Rod drives P2

        # NOTE: You will need to uncomment your marker initialization and update it here:
        # target_marker.pose = T_p2_target.A

        # 4) Try IK for Panda2
        IK_sol = panda2.ikine_LM(
            T_p2_target,
            q0=previous_q2,
            mask=[1, 1, 1, 1, 1, 1],
            ilimit=50,
            slimit=10,
        )

        # 5) Apply solution if successful
        if IK_sol.success:
            panda2.q = IK_sol.q
            previous_q2 = panda2.q.copy()

        # 6) Step simulation
        env.step(0)
        time.sleep(0.01)

        loop_count += 1

    print("Automated test path complete. Switching to manual slider control.")

    # 2. RUN MANUAL SLIDER CONTROL (INFINITE LOOP)
    while True:
        # P1's position is updated by the slider callback (set_joint function)

        # Kinematics Update
        T_p1_ee_new = panda1.fkine(panda1.q)
        T_rod_W_new = T_p1_ee_new @ T_p1_ee_to_rod_center
        rod.T = T_rod_W_new.A
        T_p2_target = T_rod_W_new @ T_rod_center_to_p2_ee_target
        # target_marker.pose = T_p2_target.A # Uncomment if target_marker is defined

        # IK Solve
        IK_sol = panda2.ikine_LM(
            T_p2_target,
            q0=previous_q2,
            mask=[1, 1, 1, 1, 1, 1],
            ilimit=50,
            slimit=10,
        )
        if IK_sol.success:
            panda2.q = IK_sol.q
            previous_q2 = panda2.q.copy()

        env.step(0)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Simulation stopped by user.")
finally:
    try:
        env.close()
    except Exception:
        pass