import numpy as np
import cv2
from picar_4wd import forward, backward, turn_left, turn_right, stop, get_distance_at
import random
import time

# Map and vehicle parameters
map_size = 100  # Size of the square map (100x100 cm)
forward_speed = 20  # Speed setting for moving forward
backward_speed = 20  # Speed setting for moving backward
turn_speed = 20  # Speed setting for turning
obstacle_map = np.zeros((map_size, map_size), dtype=np.uint8)  # Initialize the obstacle map

# Distance parameters
backup_distance = 10  # Distance to back up in cm
estimated_backup_time = backup_distance / backward_speed

# Utility function to update the obstacle grid
def interpolate_points(prev_x, prev_y, x, y, grid):
    diff = abs(y - prev_y)
    slope = (x - prev_x) / (y - prev_y) if (y - prev_y) else 0

    if slope < 0.5:  # Check if slope meets the threshold
        for j in range(diff):
            new_y = prev_y + j if y > prev_y else prev_y - j
            new_x = int(prev_x + slope * j)
            new_x, new_y = min(new_x, 99), min(new_y, 99)
            grid[new_x, new_y] = 1
    else:
        for j in range(diff):
            new_x = int(prev_x + j if x > prev_x else prev_x - j)
            new_y = int(prev_y + slope * j)
            new_x, new_y = min(new_x, 99), min(new_y, 99)
            grid[new_x, new_y] = 1

# Main function to map obstacles and control the car
def start_avoidance_and_mapping():
    print('Starting avoidance and mapping')
    prev_x = prev_y = 49  # Starting at the center bottom of the grid
    angle_step = 16  # Angle step for scanning

    while True:
        distance = get_distance_at(0)  # Distance directly ahead
        print(f"Distance: {distance}cm")

        if distance < 60:  # Too close to an obstacle
            stop()  # Stop the car
            for i in range(-60, 61, angle_step):  # Scan with an angle step
                distance = get_distance_at(i)
                angle_radians = np.radians(i)
                x = 49 + int(distance * np.sin(angle_radians))
                y = 49 + int(distance * np.cos(angle_radians))
                x, y = min(x, 99), min(y, 99)  # Clamping the values to the grid size
                obstacle_map[x, y] = 1  # Marking the obstacle on the grid
                interpolate_points(prev_x, prev_y, x, y, obstacle_map)
                prev_x, prev_y = x, y

            backup_and_turn()  # Back up and turn

        else:
            forward(forward_speed)  # Move forward if the path is clear

        time.sleep(0.5)  # Short delay to regulate loop execution

def backup_and_turn():
    backward(backward_speed)  # Move backward
    time.sleep(estimated_backup_time)  # Wait for the estimated time to back up
    stop()  # Stop before turning

    # Randomly choose a direction to turn
    random.choice([turn_left, turn_right])(turn_speed)
    time.sleep(1)  # Turn for a fixed period
    stop()  # Ensure the car stops before proceeding

if __name__ == '__main__':
    try:
        start_avoidance_and_mapping()
    except KeyboardInterrupt:
        stop()  # Ensure the car stops if the program is interrupted
    finally:
        np.save('obstacle_map.npy', obstacle_map)  # Save the numpy map
        cv2.imwrite('obstacle_map.png', obstacle_map * 255)  # Visualize and save the map
        print('Mapping completed, car stopped.')
