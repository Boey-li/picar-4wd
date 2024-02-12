import numpy as np
import cv2

# Convert target to grid units (cm)
target_x, target_y = 152, 152  # Approximating 152.4 cm for simplicity

# Initialize the map
map_size = 300  # Increase map size for this example
obstacle_map = np.zeros((map_size, map_size), dtype=np.uint8)

# Place obstacles
# For simplicity, adding a rectangular obstacle
for x in range(75, 175):
    for y in range(100, 110):
        obstacle_map[y, x] = 1



cv2.imwrite('init_map.png', obstacle_map * 255)  # Visualize and save the maps


from queue import PriorityQueue

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(map, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in neighbors(map, current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from

def neighbors(map, node):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    result = []
    for dir in directions:
        next = (node[0] + dir[0], node[1] + dir[1])
        if 0 <= next[0] < map.shape[0] and 0 <= next[1] < map.shape[1] and map[next[0], next[1]] == 0:
            result.append(next)
    return result

# Define start and goal
# Starting position and intermediate goal (5 feet forward)
start = (0, 0)
intermediate_goal = (0, 152)  # Moving forward 5 feet, so y-coordinate increases

# Final goal (from intermediate position, 5 feet to the right)
final_goal = (intermediate_goal[0] + 152, intermediate_goal[1])

# Function to trace the path from the goal back to the start
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from.get(current, start)  # Fallback to start if missing
    path.append(start)
    path.reverse()
    return path

# A* search to the intermediate goal
came_from = a_star_search(obstacle_map, start, intermediate_goal)
path_to_intermediate = reconstruct_path(came_from, start, intermediate_goal)

# Update start position for the next movement
new_start = intermediate_goal

# A* search from the intermediate position to the final goal
came_from = a_star_search(obstacle_map, new_start, final_goal)
path_to_final = reconstruct_path(came_from, new_start, final_goal)

# Combine paths for visualization
full_path = path_to_intermediate[:-1] + path_to_final  # Exclude duplicate intermediate goal
print(full_path)

# Update the map for visualization
for x, y in full_path:
    obstacle_map[y, x] = 1

cv2.imwrite('path_map.png', obstacle_map * 255)  # Visualize and save the maps
