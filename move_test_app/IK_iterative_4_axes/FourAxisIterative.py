import numpy as np
from ikpy.chain import Chain

# --- Load robot ---
chain = Chain.from_urdf_file(
    "robot_4.urdf",
    active_links_mask=[False, True, True, True, True, True]
)

# --- Animation loop ---
target_orient = [0, 0, -1]
target_orient_axis = "Z"
target_pos = [0.365+0.049, 0, 0.534]
angles = [0, 0, 0, 0, 0, 0]
prev_angles = angles

angles = chain.inverse_kinematics(
    target_position=target_pos,
    target_orientation=target_orient,
    orientation_mode=target_orient_axis,
    initial_position=prev_angles
)
