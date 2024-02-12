import numpy as np
import cv2
from picar_4wd import forward, backward, turn_left, turn_right, stop, get_distance_at
import random
import time
import math

# Map and vehicle parameters
map_size = 100  # Size of the square map (100x100 cm)
forward_speed = 20 # Speed setting for moving forward
backward_speed = 20  # Speed setting for moving backward, negative for illustration
turn_speed = 20  # Speed setting for turning
obstacle_map = np.zeros((map_size, map_size), dtype=np.uint8)  # Initialize the obstacle map

# Distance parameters
backup_distance = 10  # Distance to back up in cm
estimated_backup_time = 0.5

def get_distance(angle=0):
    """Get distance at a given angle using the ultrasonic sensor."""
    return get_distance_at(angle)

def update_map_from_scan(scan_data):
    """Update the map based on scan data collected at various angles."""
    global obstacle_map
    car_position = (map_size // 2, map_size - 1)
    for angle, distance in scan_data.items():
        if distance > 0:
            obstacle_x = int(car_position[0] + distance * np.cos(np.radians(angle)))
            obstacle_y = int(car_position[1] - distance * np.sin(np.radians(angle)))  # Inverting y-axis
            if 0 <= obstacle_x < map_size and 0 <= obstacle_y < map_size:
                obstacle_map[obstacle_y, obstacle_x] = 1  # Mark the obstacle

def scan_environment():
    """Scan the environment around the car."""
    scan_data = {}
    for angle in range(-60, 61, 30):
        distance = get_distance(angle)
        scan_data[angle] = distance
    return scan_data

def backup_and_turn():
    """Back up a short distance and then turn in a random direction."""
    backward(backward_speed)  # Move backward
    time.sleep(estimated_backup_time)  # Wait for the estimated time to back up
    stop()  # Stop before turning

    # Randomly choose a direction to turn
    random.choice([turn_left, turn_right])(turn_speed)
    time.sleep(1)  # Turn for a fixed period
    stop()  # Ensure the car stops before proceeding

def start_avoidance_and_mapping():
    """Main loop for moving the car and mapping based on detected obstacles."""
    print('Starting avoidance and mapping')
    while True:
        distance = get_distance(0)  # Distance directly ahead
        print(f"Distance: {distance}cm")

        if distance < 40:  # Too close to an obstacle
            stop()  # Stop the car
            scan_data = scan_environment()  # Scan the surroundings
            update_map_from_scan(scan_data)  # Update the obstacle map
            backup_and_turn()  # Back up and turn
        elif distance <= 50:
            stop()  # Stop before deciding the next move
            scan_data = scan_environment()  # Perform a detailed scan
            update_map_from_scan(scan_data)  # Update map with new data
            backup_and_turn()  # Execute backup and turn maneuver
        else:
            forward(forward_speed)  # Move forward if the path is clear
        time.sleep(0.5)  # Short delay to regulate loop execution

if __name__ == '__main__':
    try:
        start_avoidance_and_mapping()
    finally:
        stop()  # Ensure the car stops
        np.save('obstacle_map.npy', obstacle_map)  # Save the numpy map
        cv2.imwrite('obstacle_map.png', obstacle_map * 255)  # Visualize and save the map
        print('Mapping completed, car stopped.')
