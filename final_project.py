import numpy as np
import golfing_kinematics as kin
import time
import transforms as tf
from visualization import VizScene
import serial

# %% Color Definitions
# Define some colors for the markers
red = np.array([0.7, 0, 0, 1])
green = np.array([0, 0.7, 0, 1])
blue = np.array([0, 0, 0.7, 1])
dark_red = np.array([0.3, 0, 0, 1])
dark_green = np.array([0, 0.3, 0, 1])
dark_blue = np.array([0, 0, 0.3, 1])
white = np.array([1, 1, 1, 1])
grey = np.array([0.3, 0.3, 0.3, 1])
yellow = np.array([223.0/255.0, 238.0/255.0, 95.0/255.0, 1.0])

# %% Calculate the target position

def calculate_target_position(golf_ball_position, hole_position):
    # Step 1: Compute the direction vector
    v = hole_position - golf_ball_position

    # Step 2: Normalize the direction vector
    v_hat = v / np.linalg.norm(v)

    # Step 3: Compute the offset (0.2 units backwards)
    offset = -0.2 * v_hat

    # Step 4: Compute the new point
    new_point = golf_ball_position + offset

    return new_point

def convert_inches_to_decimeters(inches):
    for i in range(len(inches)):
        inches[i] = inches[i] * 2.54 / 10

    return inches

# %% Define the robot
d1 = 1.2675            #centimeters / 10 or decimeters
a1 = 0.15
a2 = 1.185
d4 = 1.70
tip_thickness = 0.15
dh_params = [[0, d1, a1, np.pi/2],
             [0, 0, a2, np.pi],
             [np.pi/2, 0, 0, np.pi/2],
             [0, d4, 0, 0]]

joints = ['r', 'r', 'r', 'r']
q_set1 = [0, 0, 0, 0]

R_tip = tf.roty(-np.pi/2) @ tf.rotx(np.pi/2)
T_tip = tf.se3(R_tip)
joint_limits = [[-np.pi, np.pi], [-np.pi/4, np.pi/6], [0, 3*np.pi/4], [-np.pi, np.pi]]
# arm = kin.SerialArm(dh_params, jt=joints, tip=T_tip, joint_limits=joint_limits)
arm = kin.SerialArm(dh_params, jt=joints, tip=T_tip, joint_limits=None)
golf_ball_position_inches = [5.5, -5.5, 0.]
hole_position_inches = [5.5, 0., 0.]
golf_ball_decimeters = convert_inches_to_decimeters(golf_ball_position_inches)
hole_decimeters = convert_inches_to_decimeters(hole_position_inches)
golf_ball_position = np.array([[golf_ball_decimeters[0]], [golf_ball_decimeters[1]], [golf_ball_decimeters[2]]])
hole_position = np.array([[hole_decimeters[0]], [hole_decimeters[1]], [hole_decimeters[2]]])
target_position = calculate_target_position(golf_ball_position, hole_position)
viz = VizScene()

# adding a SerialArm to the visualization, and telling it to draw the joint frames. 
viz.add_arm(arm, draw_frames=False)

# setting the joint angles to draw
viz.update(qs=[q_set1])
viz.add_marker(golf_ball_position, radius=0.2)
viz.add_marker(hole_position, radius=0.2)
viz.add_marker(target_position, radius=0.2, color=red)

viz.hold()


# %% Inverse Kinematics

vector_to_hole = hole_position - golf_ball_position
distance_to_hole = np.linalg.norm(vector_to_hole)

method = 'pinv'
K = np.array([[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]])
tol = 1e-2

q, error, count, success, message = arm.ik_position(golf_ball_position, hole_position, q_set1, method, K=K, tol=tol)

# %% Turn the tip to face the hole
import matplotlib.pyplot as plt

# Forward kinematics to get current tip position and orientation
T_tip = arm.fk(q, tip=True)
p_tip = T_tip[:3, 3]
R_tip = T_tip[:3, :3]

v_tip = R_tip[:, 0]  # First column of R_tip

# Project onto the xy-plane by setting the z-component to 0
v_projected = np.array([v_tip[0], v_tip[1], 0])

# Normalize the projected vector (optional)
v_normalized = v_projected / np.linalg.norm(v_projected)

# Define start point (p_tip in the xy-plane)
start_point = p_tip[:2]  # x, y components of p_tip

# Define end point for the line (scale the projected vector for visualization)
end_point = start_point + v_projected[:2] * 0.5  # Scale for length

v_orthogonal = np.array([-v_projected[1], v_projected[0], 0])  # Swap x and y, negate one
v_orthogonal_normalized = v_orthogonal / np.linalg.norm(v_orthogonal)  # Normalize

# Define the start and end points for the orthogonal line
orthogonal_end_point = start_point + v_orthogonal[:2] * 0.5  # Scale for visualization

# Plot both lines
# Tip position in the xy-plane
tip_xy = p_tip[:2]  # x, y components of the tip position

# Hole position in the xy-plane
hole_xy = hole_position[:2, 0]  # x, y components of the hole position

#Plot all lines
plt.figure()

# Original line (tip projection direction)
plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], 
         'r-', label="Projected Tip Direction")

# Orthogonal line
plt.plot([start_point[0], orthogonal_end_point[0]], [start_point[1], orthogonal_end_point[1]], 
         'g-', label="Orthogonal Line")

# Line from tip to hole
plt.plot([tip_xy[0], hole_xy[0]], [tip_xy[1], hole_xy[1]], 
         'b-', label="Line to Hole")

# Markers for tip and hole
plt.scatter(tip_xy[0], tip_xy[1], color='blue', label="Tip Position")
plt.scatter(hole_xy[0], hole_xy[1], color='orange', label="Hole Position")

# Labels and legend
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.grid()
plt.legend()
plt.title("Tip Projection, Orthogonal Line, and Line to Hole")
plt.axis('equal')
plt.show()

vector_to_hole_xy = hole_xy - tip_xy
vector_to_hole_normalized = vector_to_hole_xy / np.linalg.norm(vector_to_hole_xy)

# Orthogonal vector (already normalized)
orthogonal_normalized = v_orthogonal[:2] / np.linalg.norm(v_orthogonal[:2])

# Compute the cosine of the angle
cos_theta = np.dot(orthogonal_normalized, vector_to_hole_normalized)

# Ensure the value is in the range [-1, 1] to avoid numerical issues with arccos
cos_theta = np.clip(cos_theta, -1.0, 1.0)

# Compute the angle in radians and convert to degrees
theta = np.arccos(cos_theta)
theta_degrees = np.degrees(theta)

cross_product_z = orthogonal_normalized[0] * vector_to_hole_normalized[1] - orthogonal_normalized[1] * vector_to_hole_normalized[0]

# If the cross product is negative, the angle is negative
if cross_product_z < 0:
    theta = -theta

q[-1] = q[-1] - theta
adjusted_solution = q
print("Solution:", adjusted_solution)

# adjusted_solution_degrees = np.degrees(q)
adjusted_solution_degrees = np.degrees(adjusted_solution).astype(int)
adjusted_solution_degrees[-1] += 90
adjusted_solution_degrees = adjusted_solution_degrees % 360
# adjusted_solution_degrees = int(np.round((adjusted_solution_degrees + 180) % 360 - 180))
print("Solution (degrees):", adjusted_solution_degrees)


viz = VizScene()

# adding a SerialArm to the visualization, and telling it to draw the joint frames. 
viz.add_arm(arm, draw_frames=False)

# setting the joint angles to draw
viz.update(qs=[q])

viz.add_marker(golf_ball_position, radius=0.2)
viz.add_marker(hole_position, radius=0.2)

viz.hold()
viz.close_viz()


