import numpy as np
from Point import Point

class CoordinateTransform:
    def __init__(self):
        self.transform_matrix: np.ndarray = None
        self.matrix_loaded = False

    def set_transform_matrix(self, matrix):
        self.transform_matrix = matrix
        self.matrix_loaded = True

    def load_transform_matrix_from_file(self, file_path):
        try:
            loaded_matrix = np.loadtxt(file_path, delimiter=',')
            self.transform_matrix = loaded_matrix
            print(loaded_matrix)
            self.matrix_loaded = True
        except Exception as e:
            print(f"Error loading transform matrix from file: {e}")
            self.matrix_loaded = False

    def save_transform_matrix_to_file(self, file_path):
        if self.matrix_loaded:
            try:
                np.savetxt(file_path, self.transform_matrix, delimiter=',', fmt='%.6f')
                print(f"Transform matrix saved to {file_path}")
            except Exception as e:
                print(f"Error saving transform matrix to file: {e}")
        else:
            print("No transform matrix to save.")


    # Theory -> Robot coordinates = Transform matrix * Camera coordinates
    def theoretical_to_robot_coordinates(self, theoretical_coordinates : Point) -> Point:
        if not self.matrix_loaded:
            raise ValueError("Transform matrix not loaded")
        homogeneous_theoretical_coordinates = np.array([theoretical_coordinates.x,
                                                        theoretical_coordinates.y,
                                                        theoretical_coordinates.z,
                                                        1])  # Convert to homogeneous coordinates
        robot_coordinates_homogeneous = self.transform_matrix @ homogeneous_theoretical_coordinates
        return Point(*robot_coordinates_homogeneous[:3])  # Convert back to 3D coordinates
    
    def matrix_size(self):
        if self.matrix_loaded:
            return self.transform_matrix.shape
        else:
            return None