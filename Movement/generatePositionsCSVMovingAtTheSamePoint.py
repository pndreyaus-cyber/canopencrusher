import math
import numpy as np

def generate_target_positions_for_moving_at_the_same_spot():
    positions = np.array([[0+np.cos(i)*50, 414+np.sin(i)*50, 534] for i in np.linspace(0, 6*math.pi, 48)])
    np.savetxt('positions.csv', positions, delimiter=',', fmt='%d')
    
if __name__ == "__main__":
    generate_target_positions_for_moving_at_the_same_spot()


