from ikpy.chain import Chain
import math
import numpy as np

class URDF_4Axes:
    def __init__(self, urdf_path: str):
        self.urdf_path = urdf_path
        self.chain = Chain.from_urdf_file(
            urdf_path,
            active_links_mask=[False, True, True, True, True, True]
        )

        self.angles = [0, 0, 0, 3.14159/4, 3.14159/4, 0]
    
    def ik(self, prev_angles, target_pos, target_orient):
        target_orient_axis = "Z"

        angles = self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orient,
            orientation_mode=target_orient_axis,
        )

        return angles

