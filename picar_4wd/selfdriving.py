
import numpy as np
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic 
from picar_4wd.utils import *
import picar_4wd as fc
import time
import argparse
import threading

import cv2
from detector import ObjectDetector

us = Ultrasonic(Pin('D8'), Pin('D9'))
servo = Servo(PWM("P0"), offset=0)

side_length = 80
RES = 20

from pathsolver import AStarPath

class AutoPilot(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.map = np.zeros((self.height, self.width), dtype=np.uint8)

        self.cur_x = (side_length // 2)
        self.cur_y = (side_length // 2)

        # counter-clockwise is positive
        self.cur_theta = 0
    
    def get_position(self):
        return (self.cur_x, self.cur_y, self.cur_theta)
        
    def scan_surrounding(self):
        # Scan the surrounding area and return a list of (angle, distance) tuples
        # representing the obstacles
        dis = []
        for angle in range(-60, 60, 10):
            servo.set_angle(angle)
            time.sleep(0.3)
            distance = us.get_distance()
            print(distance)
            # Observations that are too far are tossed
            if distance != -2 and distance < 50:
                dis.append((angle, distance))

        servo.set_angle(0)

        return dis

    def set_obstacle(self, obs):
        # Transform to map coordinates using current position and orientation
        radians = np.deg2rad(obs[0]) + np.deg2rad(self.cur_theta)
        x = round(obs[1] * np.sin(radians) / RES)
        y = round(obs[1] * np.cos(radians) / RES)

        x = x + self.cur_x
        y = y + self.cur_y

        # Set the obstacle in the map with a radius of EPS
        EPS = 2
        for t_y in range(y - EPS, y + EPS + 1):
            for t_x in range(x - EPS, x + EPS + 1):
                if 0 <= t_x < side_length and 0 <= t_y < side_length:
                    self.map[y][x] = 1
    
    def set_surrounding(self):
        # Scan the surrounding area and set the obstacles in the map
        obstacles = self.scan_surrounding()
        for obs in obstacles:
            self.set_obstacle(obs)
    
    def get_map(self):
        return self.map
    
    def forward(self):
        # Move 15 cm forward
        fc.forward(20)
        time.sleep(0.5)
        fc.stop()
        
        x_increment = round(15 * np.sin(np.deg2rad(self.cur_theta)) / RES)
        y_increment = round(15 * np.cos(np.deg2rad(self.cur_theta)) / RES)

        self.cur_x += x_increment
        self.cur_y += y_increment

        time.sleep(0.4)

    def backward(self):
        # Move 15 cm backward
        fc.backward(20)
        time.sleep(0.5)
        fc.stop()

        x_increment = round(15 * np.sin(np.deg2rad(self.cur_theta)) / RES)
        y_increment = round(15 * np.cos(np.deg2rad(self.cur_theta)) / RES)

        self.cur_x -= x_increment
        self.cur_y -= y_increment

        time.sleep(0.4)
    
    def turn_right(self):
        # turn right for 90 degrees
        fc.turn_right(20)
        time.sleep(1.325)
        fc.stop()

        self.cur_theta -= 90
        self.cur_theta = self.cur_theta % 360

        time.sleep(0.4)
    
    def turn_left(self):
        fc.turn_left(20)
        time.sleep(1.325)
        fc.stop()

        self.cur_theta += 90
        self.cur_theta = self.cur_theta % 360

        time.sleep(0.4)
    
    def get_map_viz(self):
        ret = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        ret[self.map == 1] = (255, 0, 0) # RED for obstacle
        ret[self.map == 2] = (0, 255, 0) # GREEN for car
        return ret
    
    def get_map_viz_with_path(self, path):
        viz = self.get_map_viz()
        for i in range(len(path) - 1):
            x, y = path[i]
            viz[y, x] = (0, 0, 255)
        return viz

    def get_path(self, start, goal):
        solver = AStarPath(self.map)
        path, _ = solver.a_star_search(start, goal)
        return path

    def reconstruct_path(came_from, start, goal):
        """
        Reconstructs the path from start to goal using the came_from dictionary.
        
        :param came_from: A dictionary mapping each node to the node it came from.
        :param start: The starting node (x, y).
        :param goal: The goal node (x, y).
        :return: A list of tuples representing the path from start to goal, inclusive.
        """
        current = goal
        path = []
        while current != start:  # Loop until reaching the start node
            path.append(current)
            current = came_from[current]  # Move to the predecessor
        path.append(start)  # Add the start node
        path.reverse()  # Reverse the path to start->goal order
        return path

class CarController:
    def __init__(self, model_path, camera_id=0, frame_width=640, frame_height=480, num_threads=4, enable_edgetpu=False):
        self.auto_pilot = AutoPilot(frame_width, frame_height)
        self.object_detector = ObjectDetector(model_path, camera_id, frame_width, frame_height, num_threads, enable_edgetpu)
        self.stop_event = threading.Event()
        self.navigation_thread = threading.Thread(target=self.run_navigation)
        self.detection_thread = threading.Thread(target=self.run_detection)

    def start(self):
        self.object_detector.start()  # Start the object detection process if needed
        self.navigation_thread.start()
        self.detection_thread.start()

    def stop(self):
        self.stop_event.set()
        self.navigation_thread.join()
        self.detection_thread.join()
        cv2.destroyAllWindows()
        self.object_detector.stop()  # Stop the object detection process if needed

    def run_navigation(self):
        while not self.stop_event.is_set():
            # Perform navigation tasks (e.g., path planning, movement commands)
            self.auto_pilot.set_surrounding()
            # Add your navigation logic here, including moving the car and updating the map
            time.sleep(1)  # Simulate time delay for navigation tasks

    def run_detection(self):
        while not self.stop_event.is_set():
            # Perform object detection tasks
            frame, _ = self.object_detector.get_latest_prediction_viz()
            if frame is not None:
                cv2.imshow("Object Detection", frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
                    self.stop_event.set()
                    break
            time.sleep(0.1)  # Adjust based on your frame processing rate

if __name__ == "__main__":
    # Initialize the car controller with appropriate parameters
    car_controller = CarController(model_path="your_model_path.tflite", camera_id=0, frame_width=640, frame_height=480, num_threads=4, enable_edgetpu=False)
    try:
        car_controller.start()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        car_controller.stop()


if __name__ == '__main__':
    my_pilot = AutoPilot(side_length, side_length)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--model', help='Path of the object detection model.', required=False, default='efficientdet_lite0.tflite')
    parser.add_argument('--cameraId', help='Id of camera.', required=False, type=int, default=0)
    parser.add_argument('--frameWidth', help='Width of frame to capture from camera.', required=False, type=int, default=640)
    parser.add_argument('--frameHeight', help='Height of frame to capture from camera.', required=False, type=int, default=480)
    parser.add_argument('--numThreads', help='Number of CPU threads to run the model.', required=False, type=int, default=4)
    parser.add_argument('--enableEdgeTPU', help='Whether to run the model on EdgeTPU.', action='store_true', required=False, default=False)
    args = parser.parse_args()

    my_detector = ObjectDetector(args.model, args.cameraId, args.frameWidth, args.frameHeight, args.numThreads, args.enableEdgeTPU)

    # 30, 95
    goal = (side_length // 2 - 100 // RES, side_length // 2 + 225 // RES)

    while True:
        obj_list = my_detector.get_latest_seen_objects()
        viz, fps = my_detector.get_latest_prediction_viz()
        print(viz, fps)
        if viz is not None:
            if fps is not None:
                print(my_detector.get_latest_seen_objects())
                cv2.putText(viz, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Object Detector', viz)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
                break



        # If it sees a stop sign, stop
        if 'stop sign' in obj_list:
            time.sleep(3)
            continue
        my_pilot.set_surrounding()
        cur_x, cur_y, cur_theta = my_pilot.get_position()
        if abs(cur_x - goal[0]) < 1 and abs(cur_y - goal[1]) < 1:
            break
        path = my_pilot.get_path((cur_x, cur_y), goal)
        path_keys = list(path.keys())
        # path_keys = np.array(path_keys)

        cur_pos, next_pos = path_keys[0], path_keys[1]

        # Get target theta
        if next_pos[0] > cur_pos[0]:
            target_theta = 90
            assert next_pos[1] == cur_pos[1]
        elif next_pos[0] < cur_pos[0]:
            target_theta = 270
            assert next_pos[1] == cur_pos[1]
        elif next_pos[1] > cur_pos[1]:
            target_theta = 0
            assert next_pos[0] == cur_pos[0]
        elif next_pos[1] < cur_pos[1]:
            target_theta = 180
            assert next_pos[0] == cur_pos[0]
        else:
            raise Exception("Invalid path")
        
        # Turn if necessary
        if cur_theta != target_theta:
            # print(path[:2])
            print(path_keys[:2])
            print(f"tar: {target_theta} cur: {cur_theta}")
            if (target_theta - cur_theta) % 360 == 90:
                my_pilot.turn_left()
            elif (target_theta - cur_theta) % 360 == 270:
                my_pilot.turn_right()
            else:
                my_pilot.turn_right()
                my_pilot.turn_right()
        
        # Move forward
        my_pilot.forward()

        cur_x, cur_y, cur_theta = my_pilot.get_position()
        print(f"x: {cur_x} y: {cur_y} t: {cur_theta}")
        assert (cur_x, cur_y) == next_pos

        # Visualize map
        viz_map = my_pilot.get_map_viz_with_path(path_keys)
        # cv2.imshow('viz', viz_map)
        # cv2.waitKey(1)