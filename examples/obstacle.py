import picar_4wd as fc
import time
import random

# Speed settings
speed = 30
turn_speed = 20
reverse_speed = 20

def scan_for_obstacles():
    """
    Scan the environment and return a dictionary with distances
    at various angles.
    """
    angles = [-30, 0, 30]  # Define angles to scan
    distances = {}
    for angle in angles:
        distance = fc.get_distance_at(angle)
        distances[angle] = distance
    return distances

def decide_movement(distances):
    """
    Decide whether to move forward, turn left, turn right,
    or back up based on the distances from obstacles at various angles.
    """
    # Threshold distances to consider an obstacle too close
    threshold = 40  # centimeters
    too_close_threshold = 20  # centimeters for immediate action

    # Check for very close obstacles to back up
    if distances[0] < too_close_threshold:
        # Too close, back up
        fc.backward(reverse_speed)
        time.sleep(0.5)  # Reverse for a short period

        # Choose a random direction to turn
        if random.choice([True, False]):
            fc.turn_left(turn_speed)
        else:
            fc.turn_right(turn_speed)
        time.sleep(1)  # Turn for a short period
    elif distances[0] > threshold:
        # Path is clear, move forward
        fc.forward(speed)
    else:
        # Path is not clear, decide based on side distances
        if distances[-30] > distances[30]:
            # More space on the left, turn left
            fc.turn_left(turn_speed)
        else:
            # More space on the right or equal, turn right
            fc.turn_right(turn_speed)

def main():
    while True:
        distances = scan_for_obstacles()
        decide_movement(distances)
        time.sleep(0.1)  # Short delay to allow for sensor processing

if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()  # Ensure the car stops when the script is interrupted
