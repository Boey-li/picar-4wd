import cv2
import numpy as np

map = np.load('/home/pi/cs437/picar-4wd/picar_4wd/obstacle_map.npy')
print(np.sum(map))
cv2.imwrite('/home/pi/cs437/picar-4wd/picar_4wd/obstacle_map.png', map)