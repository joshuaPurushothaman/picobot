from motor import L298N


class Drivetrain:
    def __init__(self, left: L298N, right: L298N):
        self.left = left
        self.right = right
        self.left.set_speed(0)
        self.right.set_speed(0)

    def tankDrive(self, left: float, right: float):
        assert -1 <= left <= 1, "left must be in range -1..1"
        assert -1 <= right <= 1, "right must be in range -1..1"

        self.left.set_speed(left)
        self.right.set_speed(right)

    def arcadeDrive(self, speed: float, rotation: float):
        assert -1 <= speed <= 1, "speed must be in range -1..1"
        assert -1 <= rotation <= 1, "rotation must be in range -1..1"

        self.tankDrive(speed + rotation, speed - rotation)
