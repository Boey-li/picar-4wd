import time
from . import *

POWER_LEVEL = 50
TURN_DURATION = 1
FORWARD_DURATION = 2

def move_my(scale):
    power = POWER_LEVEL * scale
    turn_duration = TURN_DURATION * scale
    forward_duration = FORWARD_DURATION * scale
    
    forward(power)
    time.sleep(forward_duration)
    stop()
    
    turn_right(power)
    time.sleep(turn_duration)
    stop()
    
    forward(power)
    time.sleep(forward_duration)
    stop()
    
    turn_left(power)
    time.sleep(turn_duration)
    stop()
    
    forward(power)
    time.sleep(forward_duration / 2)
    stop()
    
    backward(power)
    time.sleep(forward_duration / 2)
    stop()
    
    turn_left(power)
    time.sleep(turn_duration)
    stop()
    
    backward(power)
    time.sleep(forward_duration)
    stop()
    
    turn_right(power)
    time.sleep(turn_duration)
    stop()
    
    forward(power)
    time.sleep(forward_duration)
    stop()

if __name__ == "__main__":

    speed_sensor_pin = 100
    speed_sensor = Speed(speed_sensor_pin)
    speed_sensor.start()

    try:
        move_my(scale=1)
    finally:
        speed_sensor.deinit()