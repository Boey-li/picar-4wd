import numpy as np
from queue import PriorityQueue

class AStarPath:
    def __init__(self, binary_map):
        """
        Initializes the PathFinder with a binary map.
        
        :param binary_map: A 2D numpy array where 0 represents free space and 1 represents obstacles.
        """
        self.binary_map = binary_map
        self.height, self.width = binary_map.shape

    def heuristic(self, a, b):
        """
        Calculate the Manhattan distance between two points.
        
        :param a: Tuple representing the current node.
        :param b: Tuple representing the goal node.
        :return: Manhattan distance between the two nodes.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def neighbors(self, node):
        """
        Returns valid neighbors of a node.
        
        :param node: Tuple representing the current node.
        :return: List of tuples representing valid neighbor nodes.
        """
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Possible movements: up, right, down, left
        result = []
        for dx, dy in directions:
            next_node = (node[0] + dx, node[1] + dy)
            if 0 <= next_node[0] < self.width and 0 <= next_node[1] < self.height:
                if self.binary_map[next_node[1], next_node[0]] == 0:  # Check if the node is walkable
                    result.append(next_node)
        return result

    def a_star_search(self, start, goal):
        """
        Perform A* search algorithm to find the shortest path from start to goal.
        
        :param start: Tuple representing the start node.
        :param goal: Tuple representing the goal node.
        :return: Tuple containing a dictionary of how each node was reached and the cost so far for each node.
        """
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {start: None}
        cost_so_far = {start: 0}

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next_node in self.neighbors(current):
                new_cost = cost_so_far[current] + 1  # Assuming equal cost for all moves
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    frontier.put(next_node, priority)
                    came_from[next_node] = current
                    

        return came_from, cost_so_far

# Example usage
if __name__ == "__main__":
    map_size = 100
    obstacle_map = np.zeros((map_size, map_size), dtype=np.uint8)  # Initialize map with free space

    # Define obstacles in the map
    obstacle_map[20:40, 30:50] = 1  # Example obstacle

    start = (10, 10)  # Starting position
    goal = (90, 90)  # Goal position

    pathfinder = AStarPath(obstacle_map)
    came_from, cost_so_far = pathfinder.a_star_search(start, goal)

    print(came_from, cost_so_far)
