class Point:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"Point(x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f})"
    
class SpeedAcc:
    def __init__(self, speed: float, acc: float):
        self.speed = speed
        self.acc = acc

    def __str__(self):
        return f"SpeedAcc(speed={self.speed:.5f}, acc={self.acc:.5f})"