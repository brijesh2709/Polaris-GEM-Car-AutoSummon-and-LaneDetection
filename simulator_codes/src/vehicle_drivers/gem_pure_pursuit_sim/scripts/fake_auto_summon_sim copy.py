#!/usr/bin/env python3

#================================================================
# File name: pure_pursuit_sim.py                                                                  
# Description: pure pursuit controller for GEM vehicle in Gazebo                                                              
# Author: Shrelly and Shrellyjay
# Email: sm148@illinois.edu                                                                     
# Date created: 07/1/2021                                                                
# Date last modified: 11/30/2024                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_pure_pursuit_sim pure_pursuit_sim.py                                                                    
# Python version: 3.8                                                             
#================================================================

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import NavSatFix, Imu
import alvinxy.alvinxy as axy # Import AlvinXY transformation module

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from gem_pure_pursuit_sim.msg import GpsCoordinates

class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(20)

        self.look_ahead = 2.75   # meters
        self.wheelbase  = 1.75 # meters
        self.goal       = 0
        self.offset     = 0.46 
        self.current_waypoint_index = 0
        self.csv_file_for_sensor_waypoints = "sensor_waypoints.csv"
        self.read_waypoints() # read waypoints

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0 
        self.ackermann_msg.steering_angle          = 0.0

        self.gnss_sub   = rospy.Subscriber("/gps/fix", NavSatFix, self.gps_callback)
        self.imu_sub         = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.lat        = 0.0
        self.lon        = 0.0
        self.alt         = 0.0 
        self.imu_yaw     = 0.0
        self.sensor_waypoints = []

        self.olat       = 40.0928563
        self.olon       = -88.2359994

        self.gps_lat = 0
        self.gps_lon = 0

        self.ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.auto_summon_sub = rospy.Subscriber('/auto_summon_gps_coords', GpsCoordinates, self.auto_summon_callback)
        self.auto_summon_pub = rospy.Publisher('/auto_summon_pos_reached', Bool, queue_size=1)

    def auto_summon_callback(self, msg):
        rospy.loginfo(f"Received GPS Coordinates: Latitude = {msg.latitude}, Longitude = {msg.longitude}")
        self.gps_lat = round(msg.latitude, 6)
        self.gps_lon = round(msg.longitude, 6)


    def save_sensor_waypoints(self):

        # Write the smooth waypoints to a CSV file
        with open(self.csv_file_for_sensor_waypoints, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.sensor_waypoints)  # Write data

        print(f"Waypoints have been saved to {self.csv_file_for_sensor_waypoints}")
    
    def gps_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)
        self.alt = round(msg.altitude, 6)
    
    def imu_callback(self, msg):
        orientation_q      = msg.orientation
        angular_velocity   = msg.angular_velocity
        linear_accel       = msg.linear_acceleration
        orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.imu_yaw       = round(yaw, 6)

    # def heading_to_yaw(self, heading_curr):
    #     if (heading_curr >= 270 and heading_curr < 360):
    #         yaw_curr = np.radians(450 - heading_curr)
    #     else:
    #         yaw_curr = np.radians(90 - heading_curr)
    #     return yaw_curr

    # import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/translated_coordinates.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # turn path_points into a list of floats to eliminate the need for casts
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_yaw = [float(point[2]) for point in path_points]
        self.dist_arr        = np.zeros(len(self.path_points_x))

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   
    
    def get_gem_pose(self):

        rospy.wait_for_service('/gazebo/get_model_state')
        
        try:
            service_response = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = service_response(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q      = model_state.pose.orientation
        orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        return round(x,4), round(y,4), round(yaw,4)
    
    def get_gem_pose2(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas no need as im gives yaw in the simulator
        # curr_yaw = self.heading_to_yaw(self.heading) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(self.imu_yaw)
        curr_y = local_y_curr - self.offset * np.sin(self.imu_yaw)

        return round(curr_x, 3), round(curr_y, 3), round(self.imu_yaw, 4)

    def find_target_waypoint(self, curr_x, curr_y, curr_yaw):        
        # Find the closest waypoint
        target_x, target_y, target_yaw = self.path_points_x[self.current_waypoint_index], self.path_points_y[self.current_waypoint_index], self.path_points_yaw[self.current_waypoint_index]
        
        dist_to_target = self.dist((target_x, target_y), (curr_x, curr_y))
        yaw_difference = abs(curr_yaw - target_yaw)
        yaw_difference = min(yaw_difference, 2*np.pi - yaw_difference)  # Handle angle wrapping
        
        if dist_to_target < 8: #and yaw_difference < 0.1:  # 2 meters and ~5.7 degrees
            # Look for a waypoint at the look-ahead distance
            print("heading to next way point", self.current_waypoint_index + 1, "********************************")
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.path_points_x):
                rospy.loginfo("Reached all the waypoints")
                self.save_sensor_waypoints()
                self.goal = 0
                return self.path_points_x[self.goal], self.path_points_y[self.goal], self.path_points_yaw[self.goal]
            self.current_waypoint_index %= len(self.path_points_x)

    
        # Find the next waypoint that is at least look_ahead distance away
        for idx in range(self.current_waypoint_index, len(self.path_points_x)):
                v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
                v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
                temp_angle = self.find_angle(v1,v2)
                if abs(temp_angle) < np.pi/2:
                    self.goal = idx
                    return self.path_points_x[self.goal], self.path_points_y[self.goal], self.path_points_yaw[self.goal]
        
        self.goal = -1 # If no waypoint found, return the last waypoint
        return self.path_points_x[self.goal], self.path_points_y[self.goal], self.path_points_yaw[self.goal]
    
    def get_dist_between_car_and_summon_pos(self):        
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
        local_x_gps, local_y_gps = self.wps_to_local_xy(self.gps_lon, self.gps_lat)
        return self.dist((local_x_curr, local_y_curr), (local_x_gps, local_y_gps))

    
    def start_pp(self):
        
        while not rospy.is_shutdown():
            if self.gps_lat != 0 and  self.gps_lon != 0:    

                print("error to gps auto summon point --------------------- ", self.get_dist_between_car_and_summon_pos())

                if self.get_dist_between_car_and_summon_pos() < 2:
                    self.auto_summon_pub.publish(True)
                    self.ackermann_msg.speed          = 0
                    self.ackermann_msg.steering_angle = 0
                    self.ackermann_pub.publish(self.ackermann_msg)
                    self.gps_lat = 0 
                    self.gps_lon = 0
                
                # get current position and orientation in the world frame
                curr_x, curr_y, curr_yaw = self.get_gem_pose2()
                _, _, curr_yaw = self.get_gem_pose()
                curr_sensor_waypoints = self.get_gem_pose2()
                self.sensor_waypoints.append(curr_sensor_waypoints)

                print("current x", curr_x)
                print("current y", curr_y)
                print("current yaw", curr_yaw)

                target_waypoint_x, target_waypoint_y, target_waypoint_yaw = self.find_target_waypoint(curr_x, curr_y, curr_yaw)

                self.path_points_x = np.array(self.path_points_x)
                self.path_points_y = np.array(self.path_points_y)

                # # finding the distance of each way point from the current position
                # for i in range(len(self.path_points_x)):
                #     self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))

                # # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
                # goal_arr = np.where( (self.dist_arr < self.look_ahead + 0.3) & (self.dist_arr > self.look_ahead - 0.3) )[0]

                # # finding the goal point which is the last in the set of points less than the lookahead distance
                # for idx in goal_arr:
                #     v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
                #     v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
                #     temp_angle = self.find_angle(v1,v2)
                #     if abs(temp_angle) < np.pi/2:
                #         self.goal = idx
                #         break

                # # finding the distance between the goal point and the vehicle
                # # true look-ahead distance between a waypoint and current position

                L =  self.dist((target_waypoint_x, target_waypoint_y), (curr_x, curr_y))

                print("distance to self.goal", L)
                print("x goal:", target_waypoint_x, ", y goal: ", target_waypoint_y, ", yaw goal", target_waypoint_yaw)

                # # transforming the goal point into the vehicle coordinate frame 
                # gvcx = self.path_points_x[self.goal] - curr_x
                # gvcy = self.path_points_y[self.goal] - curr_y
                # goal_x_veh_coord = gvcx*np.cos(curr_yaw) + gvcy*np.sin(curr_yaw)
                # goal_y_veh_coord = gvcy*np.cos(curr_yaw) - gvcx*np.sin(curr_yaw)

                # find the curvature and the angle 
                alpha   = target_waypoint_yaw - (curr_yaw)
                k       = 0.285
                angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / L) 
                angle   = angle_i*2
                angle   = round(np.clip(angle, -0.61, 0.61), 3)

                ct_error = round(np.sin(alpha) * L, 3)

                print("Crosstrack Error: " + str(ct_error))

                # implement constant pure pursuit controller
                self.ackermann_msg.speed          = 2.8
                self.ackermann_msg.steering_angle = angle
                self.ackermann_pub.publish(self.ackermann_msg)
                self.rate.sleep()

def pure_pursuit():

    rospy.init_node('pure_pursuit_sim_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pure_pursuit()

