#!/usr/bin/env python3

# Python Headers
import math
import time
import cv2
import numpy as np
from numpy import linalg as la
CvBridge
# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import NavSatFix, Image, Imu
from tf.transformations import euler_from_quaternion

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
import alvinxy.alvinxy as axy
from cv_bridge import CvBridge
from rrt import RRT  # Assume a simple RRT module is implemented

import utils

def transform_points_from_image_to_simulator(points, resolution, offsetX, offsetY):
    path = []
    for pathInd in range(len(points)):
        nextChasePointXTape, nextChasePointYTape = utils.transform_to_taped_coordinates(points[pathInd][1],
                                                                                  points[pathInd][0],
                                                                                  resolution)
        nextChasePointXSimulator, nextChasePointYSimulator = utils.transform_tape_to_simulator(
            nextChasePointXTape, nextChasePointYTape, offsetX, offsetY)
        path.append([nextChasePointXSimulator, nextChasePointYSimulator])
    path = np.array(path)
    return path

def pure_pursuit_control(curr_x, curr_y, curr_yaw, path, lookahead_distance=2.0):
    """
    Implements Pure Pursuit Control.
    :param curr_x: Current x-coordinate of the car
    :param curr_y: Current y-coordinate of the car
    :param curr_yaw: Current yaw of the car in radians
    :param path: List of [x, y] waypoints
    :param lookahead_distance: Distance ahead of the car to look for the target point
    :return: Steering angle and speed
    """
    closest_distance = float('inf')
    target_point = None
    wheelbase = 1.75

    # Find the target point on the path that is at least the lookahead_distance away
    for point in path:
        distance = np.sqrt((point[0] - curr_x) ** 2 + (point[1] - curr_y) ** 2)
        if distance >= lookahead_distance and distance < closest_distance:
            closest_distance = distance
            target_point = point

    if target_point is None:
        rospy.logwarn("No valid target point found! Stopping the vehicle.")
        return 0.0, 0.0  # Stop if no target point found

    # Compute the steering angle using Pure Pursuit logic
    alpha = math.atan2(target_point[1] - curr_y, target_point[0] - curr_x) - curr_yaw
    steering_angle = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead_distance)

    # Set a constant speed (you can make this dynamic if needed)
    speed = 2.0  # meters per second

    return steering_angle, speed


class Summon(object):
    def __init__(self):
        self.rate = rospy.Rate(0.5)
        self.wheelbase = 1.75  # meters

        # ROS Publishers/Subscribers
        self.ackermann_msg = AckermannDrive()
        self.ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback, queue_size=1)
        self.imu_sub         = rospy.Subscriber("/imu", Imu, self.imu_callback)

        self.rrtPlotPub = rospy.Publisher("rrt/RRTPlot", Image, queue_size=1)
        self.rrtLivePathPub = rospy.Publisher("rrt/RRTLivePath", Image, queue_size=1)

        # Variables
        self.ref_lat, self.ref_lon = 40.0928563, -88.2359994
        self.reached_dest = False

        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration = 0.0
        self.ackermann_msg.jerk = 0.0
        self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle = 0.0

        self.curr_lat = 0.0
        self.curr_lon = 0.0
        self.obstacleMap = None
        self.cvBridge = CvBridge()

        # Destination (in metric coordinates with taped start as origin)
        self.dest_gpr_coords = (40.092837, -88.235818)
        self.dest_coords = (-5.5, -21.0)#(50.0, -21.0)#(40.0928356, -88.235770)
        self.dest_x, self.dest_y = axy.ll2xy(self.dest_gpr_coords[0], self.dest_gpr_coords[1], self.ref_lat, self.ref_lon)
        self.destinationImageCoordinates = utils.transform_to_image_coordinates(self.dest_x, self.dest_y,resolution = 0.2)


    def gps_callback(self, msg):
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude

    def imu_callback(self, msg):

        orientation_q      = msg.orientation
        angular_velocity   = msg.angular_velocity
        linear_accel       = msg.linear_acceleration
        orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.imu_yaw       = yaw

   # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def get_gem_pose(self):
        """Gets the GEM car's current position and orientation."""
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            service_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = service_response(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y
        orientation_q = model_state.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        return round(x, 4), round(y, 4), round(yaw, 4)

    def plot_path(self, path):
        if self.obstacleMap is None or len(path) < 2:
            return
        obstacle_map_rgb = cv2.cvtColor(self.obstacleMap * 255, cv2.COLOR_GRAY2BGR)

        for i in range(len(path) - 1):
            cv2.line(obstacle_map_rgb, tuple(path[i][::-1]), tuple(path[i + 1][::-1]), (0, 255, 0), 2)
            cv2.circle(obstacle_map_rgb, tuple(path[i][::-1]), 3, (0, 0, 255), -1)
        obstacle_map_rgb = self.cvBridge.cv2_to_imgmsg(obstacle_map_rgb, '8UC3')
        self.voronoiMapPub.publish(obstacle_map_rgb)
        
    def plot_cv2_rrt(self, rrt, path=None):
        # Set up the canvas dimensions
        width, height = 800, 800 
        canvas = np.ones((height, width, 3), dtype=np.uint8) * 255 

        # Scale coordinates to fit within the canvas
        def scale_coordinates(x, y, x_bounds, y_bounds, width, height):
            x_scaled = int((x - x_bounds[0]) / (x_bounds[1] - x_bounds[0]) * width)
            y_scaled = int(height - (y - y_bounds[0]) / (y_bounds[1] - y_bounds[0]) * height)
            return x_scaled, y_scaled

        # Draw the RRT tree
        for node, parent in rrt.tree.items():
            if parent is not None:
                node_scaled = scale_coordinates(node[0], node[1], rrt.x_bounds, rrt.y_bounds, width, height)
                parent_scaled = scale_coordinates(parent[0], parent[1], rrt.x_bounds, rrt.y_bounds, width, height)
                cv2.line(canvas, parent_scaled, node_scaled, (255, 0, 0), 1)  # Blue dashed line (approximation)

        # Draw the start and goal points
        start_scaled = scale_coordinates(rrt.start[0], rrt.start[1], rrt.x_bounds, rrt.y_bounds, width, height)
        goal_scaled = scale_coordinates(rrt.goal[0], rrt.goal[1], rrt.x_bounds, rrt.y_bounds, width, height)
        cv2.circle(canvas, start_scaled, 5, (0, 255, 0), -1)  # Green circle for start
        cv2.circle(canvas, goal_scaled, 5, (0, 0, 255), -1)  # Red circle for goal

        # Draw the path if it exists
        if path:
            path_coords = [scale_coordinates(p[0], p[1], rrt.x_bounds, rrt.y_bounds, width, height) for p in path]
            for i in range(len(path_coords) - 1):
                cv2.line(canvas, path_coords[i], path_coords[i + 1], (0, 0, 255), 2)  # Red path line


        rrt_plot_image = self.cvBridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        self.rrtPlotPub.publish(rrt_plot_image)

        # # Add a legend
        # legend_canvas = 50  # Height of the legend space
        # canvas = cv2.copyMakeBorder(canvas, 0, legend_canvas, 0, 0, cv2.BORDER_CONSTANT, value=(255, 255, 255))
        # cv2.putText(canvas, "Start", (20, height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.putText(canvas, "Goal", (20, height + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        # cv2.putText(canvas, "Tree", (100, height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        # cv2.putText(canvas, "Path", (100, height + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # # Display the canvas
        # cv2.imshow("RRT Visualization", canvas)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()


    def start(self):
        time.sleep(1)
        while not rospy.is_shutdown():
            # Get current position
            curr_x_simulator, curr_y_simulator, curr_yaw_simulator = self.get_gem_pose()
            local_curr_x, local_curr_y = axy.ll2xy(self.curr_lat, self.curr_lon, self.ref_lat, self.ref_lon)

            # offsets change across worlds
            offsetX = local_curr_x - curr_x_simulator
            offsetY = local_curr_y - curr_y_simulator 

            # Local frame to RRT plot coordinates
            currLocationImageCoordinates = utils.transform_to_image_coordinates(local_curr_x, local_curr_y, resolution=0.2)

            print(f'---curr_x_simulator: {curr_x_simulator}')
            print(f'---curr_y_simulator: {curr_y_simulator}')
            print(f'---curr_yaw_simulator: {curr_yaw_simulator}')

            print(f'---local_curr_x: {local_curr_x}')
            print(f'---local_curr_y: {local_curr_y}')
            print(f'---dest_x: {self.dest_coords[0]}')
            print(f'---dest_y: {self.dest_coords[1]}')


            print(f'---self.curr_lat: {self.curr_lat}')
            print(f'---self.curr_lon: {self.curr_lon}')
            print(f'---self.ref_lat: {self.ref_lat}')
            print(f'---self.ref_lon: {self.ref_lon}')

            print(f'---offsetX: {offsetX}')
            print(f'---offsetY: {offsetY}')

            print(f'---currLocationImageCoordinates[::-1]: {currLocationImageCoordinates[::-1]}')
            print(f'---self.destinationImageCoordinates[::-1]: {self.destinationImageCoordinates[::-1]}')


            rrt = RRT(start=currLocationImageCoordinates[::-1], 
                    goal=self.destinationImageCoordinates[::-1],
                    x_bounds=(0,500),
                    y_bounds=(0,500),
                    max_iter=1000,
                    step_size=10)
            path = rrt.plan()

            print(f'---- path: {path}')


            # while self.obstacleMap is None:
            #     self.rate.sleep()

            # Perform RRT-based path planning
            # start = (int(curr_x_simulator), int(curr_y_simulator))
            # goal = (int(self.destinationTapeCoordinates[0]), int(self.destinationTapeCoordinates[1]))
            
            # rrt = RRT(start=start, goal=goal, obstacle_map=self.obstacleMap, max_iters=500, step_size=5)
            # path = rrt.plan_path()

            if path is None:
                rospy.loginfo("Path could not be found! Retrying...")
                continue

            # Instead of just plotting Publish it as a topic to RViz
            # self.plot_path(path)
            self.plot_cv2_rrt(rrt, path)

            PathSimulatorCoordinates = transform_points_from_image_to_simulator(points=path,resolution=0.2, 
                                                                                offsetX = offsetX, offsetY = offsetY)

            print(f'---- PathSimulatorCoordinates: {PathSimulatorCoordinates}')
            
            if self.reached_dest:
                break


            if np.any(PathSimulatorCoordinates):

                while True:
            
                    # Implement Pure Pursuit Control Here
                    # Pure Pursuit Controller
                    curr_x_simulator, curr_y_simulator, curr_yaw_simulator = self.get_gem_pose()
                    local_curr_x, local_curr_y = axy.ll2xy(self.curr_lat, self.curr_lon, self.ref_lat, self.ref_lon)

                    steering_angle, speed = pure_pursuit_control(
                        curr_x_simulator, curr_y_simulator, curr_yaw_simulator, PathSimulatorCoordinates
                    )
                    
                    distance_from_destination = self.dist((self.dest_x, self.dest_y), (local_curr_x, local_curr_y))

                    # implement constant pure pursuit controller
                    if distance_from_destination < 2.0:
                        self.ackermann_msg.speed = 0.0
                        self.ackermann_msg.steering_angle = steering_angle
                        self.ackermann_pub.publish(self.ackermann_msg)
                        self.reached_dest = True
                        print("Destination Reached!!!!!!!!!!!!!!!!!!!!!!!!!!")
                        break

                    # Publish Ackermann Drive message
                    self.ackermann_msg.steering_angle = steering_angle
                    self.ackermann_msg.speed = speed
                    self.ackermann_pub.publish(self.ackermann_msg)

                    self.rate.sleep()


def summon():
    rospy.init_node('summon_sim_node', anonymous=True)
    s = Summon()

    try:
        s.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    summon()




'''
            for point in PathSimulatorCoordinates:
                target_x, target_y = point
                distance = math.sqrt((target_x - curr_x_simulator) ** 2 + (target_y - curr_y_simulator) ** 2)
                angle_to_target = math.atan2(target_y - curr_y_simulator, target_x - curr_x_simulator)
                steering_angle = angle_to_target - curr_yaw_simulator

                # Control commands
                self.ackermann_msg.speed = 2.0 if distance > 2 else 0.0
                self.ackermann_msg.steering_angle = max(min(steering_angle, 0.61), -0.61)
                self.ackermann_pub.publish(self.ackermann_msg)

                if distance < 1.0:
                    break
                self.rate.sleep()

            rospy.loginfo("Destination Reached!")
            break
'''