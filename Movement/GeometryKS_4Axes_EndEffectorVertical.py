import math
from typing import List
try:
    from .Point import Point
except ImportError:
    from Point import Point


class GeometryKS_4Axes_EndEffectorVertical:
    def __init__(
        self,
        du: float,  # смещение по какой-то оси между 1 и 2 осями
        dv: float,  # смещение по какой-то другой оси между 1 и 2 осями
        l1: float,  # Длина первого звена (между 2 и 3 осями)
        l2: float,  # Длина второго звена (между 3 и 4 осями)
        l3: float,  # Длина третьего звена (между 4 осью и пипкой)
    ):
        self.du = du
        self.dv = dv
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def ik(
        self, point: Point
    ) -> List[
        float
    ]:  # Only works for 4 DOF robots with specific geometry, needs to be adapted for other robots
        x = point.x
        y = point.y
        z = point.z

        alpha_1 = math.atan2(x, y)

        uT = math.sqrt(x**2 + y**2)
        vT = z

        d1 = math.sqrt((uT - self.du) ** 2 + (vT + self.l3 - self.dv) ** 2)

        gamma_1 = math.acos((self.l1**2 + d1**2 - self.l2**2) / (2 * self.l1 * d1))
        gamma_2 = math.acos((self.l2**2 + self.l1**2 - d1**2) / (2 * self.l1 * self.l2))
        gamma_4 = math.acos((uT - self.du) / d1)

        alpha_2 = math.pi / 2 - gamma_1 - gamma_4
        alpha_3 = math.pi - gamma_2
        alpha_4 = math.pi - alpha_2 - alpha_3

        return [alpha_1, alpha_2, alpha_3, alpha_4]

    # def fk(
    #     self, alpha_1: float, alpha_2: float, alpha_3: float, alpha_4: float
    # ) -> Point:  # TODO: check
    #     uT = (
    #         self.du
    #         + self.l1 * math.cos(alpha_2)
    #         + self.l2 * math.cos(alpha_2 + alpha_3)
    #         + self.l3 * math.cos(alpha_2 + alpha_3 + alpha_4)
    #     )
    #     vT = (
    #         self.dv
    #         - self.l1 * math.sin(alpha_2)
    #         - self.l2 * math.sin(alpha_2 + alpha_3)
    #         - self.l3 * math.sin(alpha_2 + alpha_3 + alpha_4)
    #     )

    #     x = uT * math.sin(alpha_1)
    #     y = uT * math.cos(alpha_1)
    #     z = vT

    #     return Point(x, y, z)

    def fk(self, alpha_0: float, alpha_1: float, alpha_2: float, alpha_3: float):
        l = self.l1 * math.sin(alpha_1) + self.l2 * math.sin(alpha_1 + alpha_2)
        h = self.l1 * math.cos(alpha_1) + self.l2 * math.cos(alpha_1 + alpha_2) - self.l3

        return Point(
            l * math.sin(alpha_0) + self.du * math.sin(alpha_0),
            l * math.cos(alpha_0) + self.du * math.cos(alpha_0),
            h + self.dv,
        )