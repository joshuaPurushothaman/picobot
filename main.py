from machine import Pin, Timer

timer = Timer()
builtin_led = Pin("LED", Pin.OUT)

def blink(timer):
    builtin_led.toggle()

timer.init(freq=1, mode=Timer.PERIODIC, callback=blink)
