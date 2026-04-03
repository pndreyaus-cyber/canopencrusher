import numpy as np
import time

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

from urchin import URDF
from ikpy.chain import Chain


class URDF_Visualizer:
    def __init__(self, urdf_path):
        self.chain = Chain.from_urdf_file(urdf_path, active_links_mask=[False, True, True, True, True, True])
        self.robot = URDF.load(urdf_path)
        self.vis = meshcat.Visualizer().open()
        self.mesh_nodes = []
        self.load_meshes()

    def load_meshes(self):
        fk = self.robot.visual_trimesh_fk({j.name: 0.0 for j in self.robot.actuated_joints})
        for i, (mesh, transform) in enumerate(fk.items()):
            node_name = f"link_{i}"
            mesh_geom = self.trimesh_to_meshcat(mesh)

            self.vis[node_name].set_object(mesh_geom)
            self.vis[node_name].set_transform(transform)

            self.mesh_nodes.append((node_name, mesh))

    # --- Helper: convert trimesh → meshcat ---
    def trimesh_to_meshcat(self, mesh):
        return g.TriangularMeshGeometry(
            vertices=np.array(mesh.vertices),
            faces=np.array(mesh.faces)
        )

    # --- IK helper ---
    def ik_to_cfg(self, angles):
        return {
            joint.name: angles[i + 1]  # skip base
            for i, joint in enumerate(self.robot.actuated_joints)
        }

    def animate(self):
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

            prev_angles = angles
            angles = self.chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=target_orient,
                orientation_mode=target_orient_axis,
                initial_position=prev_angles
            )

        print(angles)

        cfg = self.ik_to_cfg(angles)
        fk = self.robot.visual_trimesh_fk(cfg)

        # 🔥 update transforms only
        for (node_name, _), (mesh, transform) in zip(self.mesh_nodes, fk.items()):
            self.vis[node_name].set_transform(transform)

        time.sleep(0.03)

if __name__ == "__main__":
    urdf_path = "urdf/ur5e.urdf"
    visualizer = URDF_Visualizer(urdf_path)
    visualizer.animate()