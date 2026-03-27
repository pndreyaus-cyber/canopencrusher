import numpy as np
import time

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

from urchin import URDF
from ikpy.chain import Chain

# --- Load robot ---
chain = Chain.from_urdf_file(
    "robot_4.urdf",
    active_links_mask=[False, True, True, True, True, True]
)
robot = URDF.load("robot_4.urdf")

# --- MeshCat viewer ---
vis = meshcat.Visualizer().open()

# --- Helper: convert trimesh → meshcat ---
def trimesh_to_meshcat(mesh):
    return g.TriangularMeshGeometry(
        vertices=np.array(mesh.vertices),
        faces=np.array(mesh.faces)
    )

# --- Load robot meshes ONCE ---
cfg = {j.name: 0.0 for j in robot.actuated_joints}
fk = robot.visual_trimesh_fk(cfg)

mesh_nodes = []

for i, (mesh, transform) in enumerate(fk.items()):
    node_name = f"link_{i}"
    mesh_geom = trimesh_to_meshcat(mesh)

    vis[node_name].set_object(mesh_geom)
    vis[node_name].set_transform(transform)

    mesh_nodes.append((node_name, mesh))

# --- IK helper ---
def ik_to_cfg(robot, angles):
    return {
        joint.name: angles[i + 1]  # skip base
        for i, joint in enumerate(robot.actuated_joints)
    }

# --- Animation loop ---
target_orient = [0, 0, -1]
target_orient_axis = "Z"
angles = [0, 0, 0, 3.14159/4, 3.14159/4, 0]

for t in np.linspace(0, 4*np.pi, 600):
    # moving target (circle)

    # Y -Z -X
    # target_pos = [0.4 + 0.25 * np.sin(t), -0.3, 0.25 * np.cos(t)]
    target_pos = [0.365+0.049, 0, 0.534]
    theta = np.pi * np.sin(t) * 0.25  # oscillates back and forth
    target_orient = [
        np.cos(theta),  
        0, 
        -np.sin(theta)
    ]

    # target_pos = [
    #     0.4 * np.cos(t),
    #     -0.2,
    #     0.3 + 0.1 * np.sin(t)
    # ]

    prev_angles = angles
    angles = chain.inverse_kinematics(
        target_position=target_pos,
        target_orientation=target_orient,
        orientation_mode=target_orient_axis,
        initial_position=prev_angles
    )

    print(angles)

    cfg = ik_to_cfg(robot, angles)
    fk = robot.visual_trimesh_fk(cfg)

    # 🔥 update transforms only
    for (node_name, _), (mesh, transform) in zip(mesh_nodes, fk.items()):
        vis[node_name].set_transform(transform)

    time.sleep(0.03)