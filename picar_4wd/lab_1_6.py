import time
import math
import numpy as np
from picar_4wd.servo import Servo
from picar_4wd import forward, backward, turn_left, turn_right, stop, get_distance_at

import random
# servo = Servo()

ANGLE_RANGE = 180
STEP = 18
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []

def get_distance(angle=0):
    return get_distance_at(angle)


obstacle_threshold_distance = 30
def scan_and_update_map():
    global obstacle_map
    scan_data = []
    for angle in range(-90, 90, STEP):
        time.sleep(0.5)
        distance = get_distance(angle)
        if distance > 0:  # Valid distance
            scan_data.append((angle, distance))
    for angle, distance in scan_data:
        if distance < obstacle_threshold_distance:
            x, y = polar_to_cartesian(angle, distance)
            if 0 <= x < map_size and 0 <= y < map_size:
                obstacle_map[x, y] = 1

def polar_to_cartesian(angle, distance):
    # Convert from polar to Cartesian coordinates
    angle_rad = math.radians(angle)
    x = int(distance * math.cos(angle_rad)) + map_size // 2  # Adjust origin to the center of the map
    y = int(distance * math.sin(angle_rad)) + map_size // 2
    return x, y

# Initialize a 100x100 map
map_size = 100
obstacle_map = np.zeros((map_size, map_size))
obstacle_distance = 30

forward_speed = 30
backward_speed = 30
turn_speed = 30

back_distance = 10
turn_distance = 20

timeout = 10
us_step = 18
max_angle = 0
min_angle = -0
force_turning = 0



obstacle_distance = 50
backward_speed = 30
turn_speed = 30
forward_speed = 30
back_up_time = 1
turn_time = 1 

def choose_random_direction():
    return random.choice([turn_left, turn_right])

def start_avoidance_and_mapping():
    print('Start avoidance and mapping')
    while True:
        distance = get_distance()
        print(f"Distance: {distance}cm")

        if distance <= obstacle_threshold_distance:
            stop()
            print("Obstacle detected, stopping.")
            scan_and_update_map()
            get_distance()
            print("Map updated, choosing new direction.")
            random_direction = choose_random_direction()
            print("Backing up.")
            backward(backward_speed)
            time.sleep(back_up_time)
            stop()  # Ensure the car stops before turning
            print("Turning.")
            random_direction(turn_speed)
            time.sleep(turn_time)
            stop()  # Ensure the car stops before moving forward
            print("Moving forward in new direction.")
            forward(forward_speed)
        else:
            forward(forward_speed)
        time.sleep(0.5)

def stop_car():
    stop()

if __name__ == '__main__':
    try:
        start_avoidance_and_mapping()
    finally:
        stop_car()
        print('Stopped')