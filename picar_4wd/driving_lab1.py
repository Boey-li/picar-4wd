import time
import random
from picar_4wd import forward, backward, turn_left, turn_right, stop, get_distance_at

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

def rand_dir():
    """Determines a random direction to turn based on the force_turning setting."""
    if force_turning == 0:
        return random.choice([turn_left, turn_right])
    elif force_turning == 1:
        return turn_left
    elif force_turning == 2:
        return turn_right

def get_distance():
    return get_distance_at(0)  # Assuming 0 is the angle facing forward

# def start_avoidance():
#     print('Start avoidance')
#     count = 0
#     while True:
#         distance = get_distance()
#         print("Distance: %scm" % distance)
#         if distance > 0:
#             count = 0
#             if distance < back_distance:
#                 print("Backward")
#                 backward(backward_speed)
#                 time.sleep(1)
#                 rand_dir()(turn_speed)
#                 time.sleep(1)
#             elif distance < turn_distance:
#                 print("Turn")
#                 rand_dir()(turn_speed)
#                 forward(forward_speed)
#                 time.sleep(1)
#             else:  # Forward
#                 forward(forward_speed)
#         else:
#             stop()
#             if count > timeout:
#                 print("Timeout reached, stopping.")
#                 break 
#             else:
#                 count += 1
#                 time.sleep(1)

obstacle_distance = 50
backward_speed = 30
turn_speed = 30
forward_speed = 30
back_up_time = 1
turn_time = 1 

def choose_random_direction():
    return random.choice([turn_left, turn_right])


def start_avoidance():
    print('Start avoidance')
    while True:
        distance = get_distance()
        print(f"Distance: {distance}cm")

        if distance <= obstacle_distance:
            stop()
            random_direction = choose_random_direction()
            print("Backing up")
            backward(backward_speed)
            time.sleep(back_up_time)
            print("Turning")
            random_direction(turn_speed)
            time.sleep(turn_time)
            print("Moving forward")
            forward(forward_speed)
        else:
            forward(forward_speed)
        time.sleep(0.5) 

def stop_car():
    stop()

if __name__ == '__main__':
    try:
        start_avoidance()
    except KeyboardInterrupt:
        stop_car()
