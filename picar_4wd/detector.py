import cv2
import numpy as np
import time
import argparse
from queue import Queue, Empty
from threading import Thread, Event
from tflite_support.task import vision
from tflite_support.task import processor
from tflite_support.task import core
import detect_utils as utils

class ObjectDetector:
    def __init__(self, model, camera_id=0, width=640, height=480, num_threads=4, enable_edgetpu=False):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.model = model
        self.enable_edgetpu = enable_edgetpu
        self.num_threads = num_threads
        self.frame_queue = Queue(maxsize=1)
        self.processed_frame_queue = Queue(maxsize=1)
        self.detection_queue = Queue(maxsize=1)
        self.fps_queue = Queue(maxsize=1)  # Queue to store FPS values
        self.stop_flag = Event()

        base_options = core.BaseOptions(file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
        detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
        options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
        self.detector = vision.ObjectDetector.create_from_options(options)

    def start(self):
        Thread(target=self.capture_frames).start()
        Thread(target=self.process_frames).start()

    def capture_frames(self):
        while not self.stop_flag.is_set():
            success, frame = self.cap.read()
            if success:
                self.frame_queue.put(frame)
            else:
                break

    def process_frames(self):
        frame_count = 0
        start_time = time.time()
        while not self.stop_flag.is_set() or not self.frame_queue.empty():
            frame = self.frame_queue.get()
            frame = cv2.flip(frame, 1)
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            input_tensor = vision.TensorImage.create_from_array(rgb_image)
            detection_result = self.detector.detect(input_tensor)

            if not self.detection_queue.full():
                self.detection_queue.put(detection_result)

            frame = utils.visualize(frame, detection_result)  # Visualization logic
            self.processed_frame_queue.put((frame, None))  # Modify to include FPS

            frame_count += 1
            if frame_count % 10 == 0:  # Calculate FPS every 10 frames
                end_time = time.time()
                fps = frame_count / (end_time - start_time)
                self.fps_queue.put(fps)  # Store the latest FPS value
                frame_count = 0
                start_time = time.time()

    def get_latest_prediction_viz(self):
        try:
            frame, _ = self.processed_frame_queue.get_nowait()
            fps = None
            try:
                fps = self.fps_queue.get_nowait()  # Attempt to get the latest FPS value
            except Empty:
                pass  # If no FPS value is available, continue without it
            return frame, fps
        except Empty:
            return None, None
        

    def get_latest_seen_objects(self):
        try:
            # return self.detection_queue.get_nowait().detections # return [Detection(bounding_box=BoundingBox(origin_x=20, origin_y=2, width=621, height=483), categories=[Category(index=0, score=0.37109375, display_name='', category_name='person')])]
            # get category name
            category_name = self.detection_queue.get_nowait().detections[0].categories[0].category_name
            return category_name
        except:
            # Return an empty list if no detections are available
            return []

    def stop(self):
        self.stop_flag.set()
        self.cap.release()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--model', help='Path of the object detection model.', required=False, default='efficientdet_lite0.tflite')
    parser.add_argument('--cameraId', help='Id of camera.', required=False, type=int, default=0)
    parser.add_argument('--frameWidth', help='Width of frame to capture from camera.', required=False, type=int, default=640)
    parser.add_argument('--frameHeight', help='Height of frame to capture from camera.', required=False, type=int, default=480)
    parser.add_argument('--numThreads', help='Number of CPU threads to run the model.', required=False, type=int, default=4)
    parser.add_argument('--enableEdgeTPU', help='Whether to run the model on EdgeTPU.', action='store_true', required=False, default=False)
    args = parser.parse_args()
    detector = ObjectDetector(args.model, args.cameraId, args.frameWidth, args.frameHeight, args.numThreads, args.enableEdgeTPU)
    detector.start()

    try:
        while True:
            viz, fps = detector.get_latest_prediction_viz()
            if viz is not None:
                if fps is not None:
                    print(detector.get_latest_seen_objects())
                    cv2.putText(viz, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow('Object Detector', viz)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC key to exit
                    break
    finally:
        detector.stop()
        cv2.destroyAllWindows()