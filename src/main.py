from machine import Pin, Timer
import utime

from drivetrain import Drivetrain
from motor import L298N

builtin_led = Pin("LED", Pin.OUT)

def linspace(a, b, n=100):
    if n < 2:
        return [b]
    diff = (float(b) - a)/(n - 1)
    return [diff * i + a  for i in range(n)]

def run():
    left = L298N(0, 1, 2)
    right = L298N(5, 3, 4)

    dt = Drivetrain(left, right)

    # Test tank drive
    nsamples = 1000
    time_s = 4

    for i in linspace(-1, 1, nsamples):
        dt.tankDrive(i, -i)
        utime.sleep_ms(int(time_s * 1000 / nsamples))

    dt.tankDrive(0,0)

if __name__ == "__main__":
    run()