import sys
import numpy as np
import math
from picar_4wd.pwm import PWM
from picar_4wd.adc import ADC
from picar_4wd.pin import Pin
from picar_4wd.motor import Motor
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic 
from picar_4wd.speed import Speed
from picar_4wd.filedb import FileDB  
from picar_4wd.utils import *
import picar_4wd as fc
import time
import cv2

config = FileDB("config")
left_front_reverse = config.get('left_front_reverse', default_value = False)
right_front_reverse = config.get('right_front_reverse', default_value = False)
left_rear_reverse = config.get('left_rear_reverse', default_value = False)
right_rear_reverse = config.get('right_rear_reverse', default_value = False)    
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 

us = Ultrasonic(Pin('D8'), Pin('D9'))
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

side_length = 200

class Map2:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.map = np.zeros((self.height, self.width), dtype=np.uint8)
    
    def get_map(self):
        return self.map
    
    def read_distance(self):
      return us.get_distance()
    
    def scan_surrounding(self):
        dis = []
        for angle in range(-60, 60, 10):
            servo.set_angle(angle)
            time.sleep(0.3)
            distance = self.read_distance()
            print(distance)
            # Observations that are too far are tossed
            if distance != -2 and distance < 50:
                dis.append((angle, distance))

        return dis

    def check_obstacle(self, obs):
        global side_length

        radians = np.deg2rad(obs[0])
        x = round(obs[1] * np.sin(radians))
        y = round(obs[1] * np.cos(radians))
        x = x + math.floor(side_length / 2)

        if 0 <= x < side_length and 0 <= y < side_length:
            self.set_obstacle(x, y)

    def set_obstacle(self, x, y):
        RES = 5

        self.map[y // RES][x // RES] = 1
    
    def set_surrounding(self):
        obstacles = self.scan_surrounding()
        for obs in obstacles:
            self.check_obstacle(obs) 



    

if __name__ == '__main__':  
    my_map = Map2(200, 200)
    my_map.set_surrounding()
    grid = my_map.get_map()
    cv2.imwrite('map.png', grid)