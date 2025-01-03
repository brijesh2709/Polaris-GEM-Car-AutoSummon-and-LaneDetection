import numpy as np
import cv2

class RRT:
    def __init__(self, start, goal, x_bounds, y_bounds, step_size, max_iter):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.max_iter = max_iter
        self.tree = {tuple(start): None}

    def random_point(self):
        x = np.random.uniform(self.x_bounds[0], self.x_bounds[1])
        y = np.random.uniform(self.y_bounds[0], self.y_bounds[1])
        return np.array([x, y])

    def nearest_neighbor(self, point):
        return min(self.tree.keys(), key=lambda n: np.linalg.norm(np.array(n) - point))

    def steer(self, from_node, to_point):
        direction = to_point - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return to_point
        return np.array(from_node) + (direction / distance) * self.step_size

    def is_goal_reached(self, node):
        return np.linalg.norm(np.array(node) - self.goal) <= self.step_size

    def plan(self):
        for _ in range(self.max_iter):
            rand_point = self.random_point()
            nearest_node = self.nearest_neighbor(rand_point)
            new_node = self.steer(nearest_node, rand_point)
            self.tree[tuple(new_node)] = nearest_node

            if self.is_goal_reached(new_node):
                self.tree[tuple(self.goal)] = tuple(new_node)
                return self.construct_path()

        return None

    def construct_path(self):
        path = [tuple(self.goal)]
        while path[-1] is not None:
            path.append(self.tree[path[-1]])
        return path[::-1][1:] 


    def plot_rrt(self, rrt, path=None):
        # Set up the canvas dimensions
        width, height = 800, 800 
        canvas = np.ones((height, width, 3), dtype=np.uint8) * 255 

        # Scale coordinates to fit within the canvas
        def scale_coordinates(x, y, x_bounds, y_bounds, width, height):
            x_scaled = int((x - x_bounds[0]) / (x_bounds[1] - x_bounds[0]) * width)
            y_scaled = int(height - (y - y_bounds[0]) / (y_bounds[1] - y_bounds[0]) * height)
            return x_scaled, y_scaled

        for node, parent in rrt.tree.items():
            if parent is not None:
                node_scaled = scale_coordinates(node[0], node[1], rrt.x_bounds, rrt.y_bounds, width, height)
                parent_scaled = scale_coordinates(parent[0], parent[1], rrt.x_bounds, rrt.y_bounds, width, height)
                cv2.line(canvas, parent_scaled, node_scaled, (255, 0, 0), 1) 

        start_scaled = scale_coordinates(rrt.start[0], rrt.start[1], rrt.x_bounds, rrt.y_bounds, width, height)
        goal_scaled = scale_coordinates(rrt.goal[0], rrt.goal[1], rrt.x_bounds, rrt.y_bounds, width, height)
        cv2.circle(canvas, start_scaled, 5, (0, 255, 0), -1) 
        cv2.circle(canvas, goal_scaled, 5, (0, 0, 255), -1)  

        if path:
            path_coords = [scale_coordinates(p[0], p[1], rrt.x_bounds, rrt.y_bounds, width, height) for p in path]
            for i in range(len(path_coords) - 1):
                cv2.line(canvas, path_coords[i], path_coords[i + 1], (0, 0, 255), 2) 

        cv2.imwrite("rrt_plot.png", canvas)
        return canvas