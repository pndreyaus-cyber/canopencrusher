from numpy import matmul
from serialRobotClient import SerialRobotClient
from ik1 import calculate_angles
import time

transform_matrix = [[0.982685, -0.02253, 0.022949, 30.20585], # Changed -21.7568 to -17.7568
                    [-0.00843, 0.939357, 0.000573, 559.0083], # Changed 542.5955 to 546.5955
                    [-0.00948, -0.01456, 1.032103, 281.2634]]

def calculate_real_position(position_theory: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(matmul(transform_matrix, [*position_theory, 1]))

def check_reply(reply: str | None) -> bool:
    if reply is None:
        print("No reply received")
        return False
    blocks = reply.split()
    if blocks[1] != "OK":
        print(f"Reply : '{reply}' failed")
        return False
    return True

def make_move(client: SerialRobotClient, 
              cube_position_theory_plane: tuple[float, float],
              cube_hover_dist: float,
              cube_pick_dist: float,
              drop_point_theory: tuple[float, float, float],
              drop_hover_dist: float,
              drop_place_dist: float,
              cubes_filtered: int):
    # Step 1
    cube_height = 25.0
    cube_position_theory = (*cube_position_theory_plane, cube_height)

    # Step 2
    cube_position_real = calculate_real_position(cube_position_theory)
    print("Real position: ", cube_position_real)
    
    #Step 3 - move above the cube
    first_position = (cube_position_real[0], cube_position_real[1], cube_position_real[2] + cube_hover_dist)
    first_position_angles = calculate_angles(*first_position, 49, 286, 372, 365.5, 115.5)
    client.move_to(first_position_angles, 0.15, 0.02)
    reply = client.wait_for_prefix("MAP", 5.0)
    if check_reply(reply):
        print("Successfully moved above the cube")
    else:
        return False
    
    if input("Continue to move down? (y/n):") != "y":
        return False

    # Step 4 - move down
    second_position = (cube_position_real[0], cube_position_real[1], cube_position_real[2] - cube_pick_dist)
    second_position_angles = calculate_angles(*second_position, 49, 286, 372, 365.5, 115.5)
    client.move_to(second_position_angles, 0.07, 0.01)
    reply = client.wait_for_prefix("MAP", 5.0)
    if check_reply(reply):
        print("Successfully moved down to the cube")
    else:
        return False
    
    # if input("Continue to pick up the cube? (y/n):") != "y":
    #     return False
    time.sleep(0.5)    

    # Step 5 - pick up the cube (move up)
    client.send_command("GRB")
    reply = client.wait_for_prefix("GRB", 5.0)
    if check_reply(reply):
        print("Successfully picked up the cube")
    else:
        return False
    
    # if input("Continue to move up with the cube? (y/n):") != "y":
    #     return False
    time.sleep(0.5)
    
    # Step 6 - move up
    client.move_to(first_position_angles, 0.15, 0.02)
    reply = client.wait_for_prefix("MAP", 5.0)
    if check_reply(reply):
        print("Successfully moved up with the cube")
    else:
        return False
    
    if input("Continue to move to the drop position? (y/n):") != "y":
        return False
    
    # Step 7 - move to the drop position
    drop_position_real = calculate_real_position(drop_point_theory)
    top_of_the_current_cube_in_pile = drop_position_real[2] + cube_height * (cubes_filtered + 1)
    first_position = (drop_position_real[0],
                      drop_position_real[1],
                      max(cube_position_real[2] + cube_hover_dist, top_of_the_current_cube_in_pile + drop_hover_dist))
    
    first_position_angles = calculate_angles(*first_position, 49, 286, 372, 365.5, 115.5)
    client.move_to(first_position_angles, 0.15, 0.02)
    reply = client.wait_for_prefix("MAP", 5.0)
    if check_reply(reply):
        print("Successfully moved to the drop position")
    else:
        return False
    
    # if input("Continue to move down to the drop position? (y/n):") != "y":
    #     return False
    time.sleep(0.5)

    # Step 8 - move down
    second_position = (drop_position_real[0], drop_position_real[1], top_of_the_current_cube_in_pile - drop_place_dist)
    second_position_angles = calculate_angles(*second_position, 49, 286, 372, 365.5, 115.5)
    client.move_to(second_position_angles, 0.07, 0.01)
    reply = client.wait_for_prefix("MAP", 5.0)
    if check_reply(reply):
        print("Successfully moved down to the drop position")
    else:
        return False
    
    # if input("Continue to drop the cube? (y/n):") != "y":
    #     return False
    time.sleep(0.5)

    # Step 9 - drop the cube    
    client.send_command("LGO")
    reply = client.wait_for_prefix("LGO", 5.0)
    if check_reply(reply):
        print("Successfully dropped the cube")
    else:
        return False
    
    # if input("Continue to move up after dropping the cube? (y/n):") != "y":
    #     return False
    time.sleep(0.5)

    # Step 10 - move up
    client.move_to(first_position_angles, 0.15, 0.02)
    reply = client.wait_for_prefix("MAP", 5.0)
    if check_reply(reply):
        print("Successfully moved up after dropping the cube")
    else:
        return False

    
    print("Move completed successfully")
    return True