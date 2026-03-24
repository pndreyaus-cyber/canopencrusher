import math
from typing import List, Tuple

class IkParameters:
    def __init__(self, robotX = 0, robotY = 0, robotZ = 0, d0=0.115, d1=0.46, d2=0.315, d3=0.092, angleZeroDirection=1.0, angleZeroAdd=0.0,
                 angleOneDirection=1.0, angleTwoDirection=1.0, angleThreeDirection=1.0):
        self.robotX = robotX
        self.robotY = robotY
        self.robotZ = robotZ
        self.d0 = d0
        self.d1 = d1
        self.d2 = d2
        self.d3 = d3
        self.joint_num = 4
        self.angleZeroDirection = angleZeroDirection
        self.angleZeroAdd = angleZeroAdd
        self.angleOneDirection = angleOneDirection
        self.angleTwoDirection = angleTwoDirection
        self.angleThreeDirection = angleThreeDirection

def calc_ik_simple(goalX: float, goalY: float, goalZ: float, ikParams: IkParameters) -> Tuple[bool, List[float]]:
    # Initialize joint angles with zeros
    num_joints = ikParams.joint_num
    jointGroupPositions = [0.0] * num_joints

    # Project goal into the plane of the first joint
    localX = math.sqrt((ikParams.robotX - goalX)**2 + (ikParams.robotY - goalY)**2)
    localY = goalZ - ikParams.d0 - ikParams.robotZ
    distance = math.sqrt(localX**2 + localY**2)

    print("localX=", localX)
    print("localY=", localY)
    print("distance=", distance)

    # Base rotation
    jointGroupPositions[0] = (
        ikParams.angleZeroDirection * math.atan2(goalY - ikParams.robotY, goalX - ikParams.robotX)
        + ikParams.angleZeroAdd
    )

    print("joint 0 angle (rad)=", jointGroupPositions[0])

    # Check if the target is reachable
    if distance > ikParams.d3 and distance < ikParams.d1 + ikParams.d2 + ikParams.d3:
        print("Target is reachable, calculating IK...")
        b1 = (ikParams.d1**2 - ikParams.d3**2 + (distance - ikParams.d2)**2) / (2 * (distance - ikParams.d2))
        b3 = distance - b1 - ikParams.d2
        print("b1=", b1)
        print("b3=", b3)

        gamma = math.asin(localY / distance)
        print("gamma (rad)=", gamma)
        print("b1 / ikParams.d1=", b1 / ikParams.d1, b1, ikParams.d1)
        jointGroupPositions[1] = math.asin(b1 / ikParams.d1)
        jointGroupPositions[2] = ikParams.angleTwoDirection * (
            math.pi / 2 - jointGroupPositions[1]
        )
        print("joint 1 angle (rad) before direction adjustment=", jointGroupPositions[1])
        print("joint 2 angle (rad)=", jointGroupPositions[2])
        jointGroupPositions[1] = ikParams.angleOneDirection * (
            jointGroupPositions[1] - gamma
        )
        print("joint 1 angle (rad) after direction adjustment=", jointGroupPositions[1])

        jointGroupPositions[3] = ikParams.angleThreeDirection * math.acos(b3 / ikParams.d3)
        print("joint 3 angle (rad)=", jointGroupPositions[3])
        return True, jointGroupPositions
    else:
        return False, jointGroupPositions


def calculate_angles(x: float, y: float, z: float, du: float, dv: float, l1: float, l2: float, l3: float):
    alpha_1 = math.atan2(x, y)

    uT = math.sqrt(x**2 + y**2)
    vT = z

    d1 = math.sqrt((uT - du)**2 + (vT + l3 - dv)**2)

    gamma_1 = math.acos((l1**2 + d1**2 - l2**2)/(2*l1*d1))
    gamma_2 = math.acos((l2**2 + l1**2 - d1**2)/(2*l1*l2))
    gamma_4 = math.acos((uT - du)/d1)

    alpha_2 = math.pi/2 - gamma_1 - gamma_4
    alpha_3 = math.pi - gamma_2
    alpha_4 = math.pi - alpha_2 - alpha_3

    return [alpha_1, alpha_2, alpha_3, alpha_4]
