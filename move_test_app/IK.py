import numpy as np

class FourAxisSolver: 
    def __init__(
            self, 
            base_coord=[0., 0., 286.], 
            lengths={"lower": 372.1, "upper": 365.5, "tool": 124, "base_forward_offset": 49}, 
            home_angles={"base": 0, "upper": 0, "lower": 0, "tool_x": 0, "tool_y": 0}
        ):
        self.base_coord = np.array(base_coord)
        self.lengths = lengths
        self.home_angles = home_angles
        self.lower_vec = np.array([0., 0., self.lengths['lower']])
        self.upper_vec = np.array([0., self.lengths['upper'], 0.])
        self.tool_vec = np.array([0., self.lengths['tool'], 0.])

        
    def rad(self, dg):
        return np.radians(dg)


    def deg(self, rd):
        return np.rad2deg(rd)


    def normalize(self, vector):
        return vector / np.linalg.norm(vector)


    def length(self, vector):
        x, y, z = vector
        return np.sqrt(x**2 + y**2 + z**2)


    def angle_between(self, vector1, vector2):
        return self.deg(np.arccos(np.dot(self.normalize(vector1), self.normalize(vector2))))


    def rotation(self, vector, axis, theta):
        theta = self.rad(theta)
        axis = np.asarray(axis)
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.dot(vector, np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]]))
    

    def solve(self, target, x_angle=0, z_angle=0):
        # print("target: [{0}, {1}, {2}] angles: x: {3}, y: {4}\n".format(*target, x_angle, z_angle))
        z_angle += 180
        tool_vec_inv = self.tool_vec
        tool_vec_inv = self.rotation(tool_vec_inv, [0, 0, 1], z_angle)
        tool_vec_inv = self.rotation(tool_vec_inv, [1, 0, 0], x_angle)
        upper_point_inv = target + tool_vec_inv
        base_rotation = [upper_point_inv[0], upper_point_inv[1], self.base_coord[2]] - self.base_coord
        base_rotation = self.deg(np.arctan(base_rotation[0]/base_rotation[1]))
        arm_joints_axis = self.rotation([1, 0, 0], [0, 0, 1], base_rotation)
        arm_vector = upper_point_inv - self.base_coord - self.rotation([0, self.lengths['base_forward_offset'], 0], [0, 0, 1], base_rotation)
        arm_l = self.length(arm_vector)
        base_angle = self.deg(np.arccos((arm_l**2 + self.lengths['lower']**2 - self.lengths['upper']**2)/(2.0*arm_l*self.lengths['lower'])))
        arm_angle = self.angle_between(arm_vector, np.array([arm_vector[0], arm_vector[1], 0]))
        base_angle += arm_angle - 90
        base_angle *= -1
        lower_vec_inv = self.rotation(self.lower_vec, arm_joints_axis, base_angle)
        lower_point_inv = self.base_coord + lower_vec_inv
        lower_angle = self.deg(np.arccos((self.lengths['upper']**2+self.lengths['lower']**2-arm_l**2)/(2.0*self.lengths['upper']*self.lengths['lower'])))
        upper_vec_inv = upper_point_inv-lower_point_inv
        tool_x_angle = self.angle_between(upper_vec_inv, np.array([upper_vec_inv[0], upper_vec_inv[1], 0])) - x_angle
        tool_z_angle = self.angle_between(np.array([tool_vec_inv[0], tool_vec_inv[1], 0]),
                                    np.array([upper_vec_inv[0], upper_vec_inv[1], 0]))

        # if round(length(upper_point_inv-lower_point_inv)) != lengths['upper']:
        #     print('CALCULATION ERROR')
        # ik_points = [target, upper_point_inv, lower_point_inv, base_coord, [0, 0, 0]]
        # ik_angles = [z_angle, x_angle, base_rotation, base_angle, lower_angle, tool_x_angle, tool_z_angle]
        # ik_angles = [ base_rotation, base_angle, 180-lower_angle, 180+tool_x_angle]

        ik_angles = [ base_rotation, base_angle, 180-lower_angle, base_angle - lower_angle + 180]

        return  ik_angles


def main():
    solver = FourAxisSolver()
    # target = [0, 365+49, 534]
    target = [0, 307, 792.2]
    angles = solver.solve(target, 90, 0)
    print(angles)


if __name__ == '__main__':
    main()