from machine import Pin, PWM
import math

class L298N:
    # def __init__(self, ena: int, in1:int, in2:int):
    def __init__(self, ena, in1, in2):
        assert ena in range(0, 29+1), "ena must be in range 0..29"
        assert in1 in range(0, 29+1), "in1 must be in range 0..29"
        assert in2 in range(0, 29+1), "in2 must be in range 0..29"

        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        
        self.ena = PWM(Pin(ena))  # type: ignore
        # https://learn.adafruit.com/improve-low-speed-performance-of-brushed-dc-motors
        # hence the freq=50
        self.ena.freq(50)

    def set_speed(self, speed: float):
        assert -1 <= speed <= 1, "speed must be in range -1..1"

        if speed > 0:
            self.in1.value(1)
            self.in2.value(0)
        elif speed < 0:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.in1.value(1)
            self.in2.value(1)

        self.ena.duty_u16(int(math.fabs(speed) * 65535))


class SmartMotor:
    """
    - L298N
    - SimpleMotorFeedForward
    - ProfiledPID or at least PID
    """
    def __init__(self, ena: int, in1:int, in2:int):
        self.l298n = L298N(ena, in1, in2)

    def set_speed_raw(self, speed: float):
        self.l298n.set_speed(speed)

    