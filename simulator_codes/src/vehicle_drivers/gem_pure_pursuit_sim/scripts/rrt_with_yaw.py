import numpy as np
import cv2

class RRTWithYaw:
    def __init__(self, start, goal, x_bounds, y_bounds, step_size, max_iter, yaw_step=np.pi / 8):
        self.start = np.array(start)  # start = [x, y, yaw]
        self.goal = np.array(goal)  # goal = [x, y, yaw]
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.step_size = step_size
        self.yaw_step = yaw_step
        self.max_iter = max_iter
        self.tree = {tuple(start): None}

    def random_point(self):
        x = np.random.uniform(self.x_bounds[0], self.x_bounds[1])
        y = np.random.uniform(self.y_bounds[0], self.y_bounds[1])
        yaw = np.random.uniform(-np.pi, np.pi)
        return np.array([x, y, yaw])

    def nearest_neighbor(self, point):
        return min(
            self.tree.keys(),
            key=lambda n: np.linalg.norm(np.array(n[:2]) - point[:2]) + abs(n[2] - point[2])
        )

    def steer(self, from_node, to_point):
        from_node = np.array(from_node)
        direction = to_point[:2] - from_node[:2]
        distance = np.linalg.norm(direction)
        yaw_diff = to_point[2] - from_node[2]
        
        if distance < self.step_size and abs(yaw_diff) < self.yaw_step:
            return to_point

        new_x, new_y = from_node[:2] + (direction / distance) * min(distance, self.step_size)
        new_yaw = from_node[2] + np.clip(yaw_diff, -self.yaw_step, self.yaw_step)
        return np.array([new_x, new_y, new_yaw])

    def is_goal_reached(self, node):
        return (
            np.linalg.norm(np.array(node[:2]) - self.goal[:2]) <= self.step_size
            and abs(node[2] - self.goal[2]) <= self.yaw_step
        )

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

    def plot_rrt(self, path=None):
        # Create a blank canvas
        img_width, img_height = 800, 800
        canvas = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255

        # Scale coordinates to fit the canvas
        def scale_coords(coords):
            x = int((coords[0] - self.x_bounds[0]) / (self.x_bounds[1] - self.x_bounds[0]) * img_width)
            y = int((coords[1] - self.y_bounds[0]) / (self.y_bounds[1] - self.y_bounds[0]) * img_height)
            return x, img_height - y

        # Plot tree edges
        for node, parent in self.tree.items():
            if parent is not None:
                node_coords = scale_coords(node[:2])
                parent_coords = scale_coords(parent[:2])
                cv2.line(canvas, node_coords, parent_coords, (200, 200, 200), 1)

        # Plot yaw arrows
        for node in self.tree.keys():
            x, y = scale_coords(node[:2])
            yaw = node[2]
            arrow_end = (int(x + 15 * np.cos(yaw)), int(y - 15 * np.sin(yaw)))
            cv2.arrowedLine(canvas, (x, y), arrow_end, (0, 0, 255), 1, tipLength=0.3)

        # Plot start and goal
        start_coords = scale_coords(self.start[:2])
        goal_coords = scale_coords(self.goal[:2])
        cv2.circle(canvas, start_coords, 5, (0, 255, 0), -1)  # Start in green
        cv2.circle(canvas, goal_coords, 5, (0, 0, 255), -1)  # Goal in red

        # Plot path
        if path:
            for i in range(len(path) - 1):
                node_coords = scale_coords(path[i][:2])
                next_node_coords = scale_coords(path[i + 1][:2])
                cv2.line(canvas, node_coords, next_node_coords, (0, 0, 255), 2)
        cv2.imwrite("rrt_plot.png", canvas)
        return canvas