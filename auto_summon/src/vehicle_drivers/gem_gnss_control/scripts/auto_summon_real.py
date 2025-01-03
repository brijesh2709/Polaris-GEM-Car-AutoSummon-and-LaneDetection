#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                          
# Version: 1.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva
from sensor_msgs.msg import NavSatFix, Image
from septentrio_gnss_driver.msg import INSNavGeod

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt
from gem_gnss_control.msg import GpsCoordinates
from rrt import RRT  # Assume a simple RRT module is implemented
import utils
from cv_bridge import CvBridge



class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0

    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted


class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(10)

        self.look_ahead = 4
        self.wheelbase  = 2.57 # meters
        self.offset     = 1.26 # meters
        self.current_waypoint_index = 0


        # self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        self.gnss_sub   = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gnss_callback)
        self.ins_sub    = rospy.Subscriber("/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)
        self.rrtPlotPub = rospy.Publisher("rrt/RRTPlot", Image, queue_size=1)

        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.auto_summon_sub = rospy.Subscriber('/auto_summon_gps_coords', GpsCoordinates, self.auto_summon_callback)
        self.auto_summon_pub = rospy.Publisher('/auto_summon_pos_reached', Bool, queue_size=1)
        self.speed      = 0.0
        self.cvBridge = CvBridge()
        self.olat       = 40.092855    
        self.olon       = -88.235981 
        self.path_points_x   = []
        self.path_points_y   = []
        self.rrt_plot_image = None
        self.path_points_yaw = []
        self.gps_lat = 0
        self.gps_lon = 0
        self.rrt_path_found = False
        # read waypoints into the system 
        self.goal       = 0            
        # self.read_waypoints() 

        self.desired_speed = 1.5  # m/s, reference speed
        self.max_accel     = 0.5 # % of acceleration
        self.pid_speed     = PID(0.5, 0.0, 0.1, wg=20)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)

        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = False

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 3.5 # radians/second

    def calculate_yaw_for_path(self, waypoints):
        yaws = []
        for i in range(len(waypoints) - 1):
            curr_x, curr_y = waypoints[i]
            next_x, next_y = waypoints[i + 1]
            yaw = math.atan2(next_y - curr_y, next_x - curr_x)
            yaws.append(yaw)
        return yaws
    
    def auto_summon_callback(self, msg):
        rospy.loginfo(f"Received GPS Coordinates: Latitude = {msg.latitude}, Longitude = {msg.longitude}")
       
        rospy.loginfo(f"Planning Path with RRT")
        # convert gps coords to local xy and transform to image coords
        local_x_gps, local_y_gps = self.wps_to_local_xy(msg.longitude, msg.latitude)

        # convert points to image coords with resolution
        x_dest_img = np.floor(local_x_gps / 0.2).astype(np.int32)
        y_dest_img = np.floor(-local_y_gps / 0.2).astype(np.int32)
        x_dest_img += int(np.floor(20 / 0.2))
        y_dest_img += int(np.floor(10 / 0.2))

        destLocationImageCoordinates = (x_dest_img, y_dest_img)


        # convert cars current coords to local xy and transform to image coords
        local_curr_x, local_curr_y = self.wps_to_local_xy(self.lon, self.lat)

        x_curr_img = np.floor(local_curr_x / 0.2).astype(np.int32)
        y_curr_img = np.floor(-local_curr_y / 0.2).astype(np.int32)
        x_curr_img += int(np.floor(20 / 0.2))
        y_curr_img += int(np.floor(10 / 0.2))

        currLocationImageCoordinates = (x_curr_img, y_curr_img)
        
        while self.rrt_path_found == False:
            rrt = RRT(start=[currLocationImageCoordinates[1], currLocationImageCoordinates[0]], 
                        goal=[destLocationImageCoordinates[1], destLocationImageCoordinates[0]],
                        x_bounds=(55,120),
                        y_bounds=(20,330),
                        max_iter=1000,
                        step_size=10)
            path = rrt.plan()
            if path!=None:
                print("Path found")
                canvas = rrt.plot_rrt(rrt, path)
                self.rrt_plot_image = self.cvBridge.cv2_to_imgmsg(canvas, encoding='bgr8')
                self.rrtPlotPub.publish(self.rrt_plot_image)
                self.rrt_path_found = True

                path_points = self.transform_points_img_to_sim(points=path)

                self.path_points_x   = [float(point[0]) for point in path_points]
                self.path_points_y   = [float(point[1]) for point in path_points]
                self.path_points_yaw = self.calculate_yaw_for_path(path_points)
                self.dist_arr        = np.zeros(len(self.path_points_x))
                self.gps_lat = round(msg.latitude, 6)
                self.gps_lon = round(msg.longitude, 6)

                to_write_to_file = []
                for a, b in zip(path_points, self.path_points_yaw):
                    to_write_to_file.append([a[0], a[1], b])

                with open("gen_waypoints.csv", mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerows(to_write_to_file)  # Write data

                print(f"Generated Waypoints have been saved to gen_waypoints.csv")


    def ins_callback(self, msg):
        self.heading = round(msg.heading, 6)

    def gnss_callback(self, msg):
        self.lat = round(msg.latitude, 6)
        self.lon = round(msg.longitude, 6)

    def speed_callback(self, msg):
        self.speed = round(msg.vehicle_speed, 3) # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def heading_to_yaw(self, heading_curr):
        if (heading_curr >= 270 and heading_curr < 360):
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    def front2steer(self, f_angle):
        if(f_angle > 35):
            f_angle = 35
        if (f_angle < -35):
            f_angle = -35
        if (f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0
        return steer_angle


    def wps_to_local_xy(self, lon_wp, lat_wp):
        # convert GNSS waypoints into local fixed frame reprented in x and y
        lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
        return lon_wp_x, lat_wp_y   
    
    def transform_points_img_to_sim(self, points, offsetX=0, offsetY=0):
        path = []
        for pathInd in range(len(points)):
            x_img = np.floor(points[pathInd][1] / 0.2).astype(np.int32)
            y_img = np.floor(-points[pathInd][0] / 0.2).astype(np.int32)

            x_img += int(np.floor(20 / 0.2))
            y_img += int(np.floor(10 / 0.2))
 
            next_x_wpt = x_img - offsetX
            next_y_wpt = y_img - offsetY

            path.append([next_x_wpt, next_y_wpt])
        
        path = np.array(path)
        return path


    def find_target_waypoint(self, curr_x, curr_y, curr_yaw):        
        # Find the closest waypoint
        target_x, target_y, target_yaw = self.path_points_x[self.current_waypoint_index], self.path_points_y[self.current_waypoint_index], self.path_points_yaw[self.current_waypoint_index]
        
        dist_to_target = self.dist((target_x, target_y), (curr_x, curr_y))
        
        if dist_to_target < 4:
            self.current_waypoint_index += 1
            if self.current_waypoint_index == len(self.path_points_x):
                rospy.loginfo("Reached all the waypoints")
                self.goal = -1
                return self.path_points_x[self.goal], self.path_points_y[self.goal], self.path_points_yaw[self.goal]
    
        # Find the next waypoint that is at least look_ahead distance away
        for idx in range(self.current_waypoint_index, len(self.path_points_x)-1):
                v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
                v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
                temp_angle = self.find_angle(v1,v2)
                if abs(temp_angle) < np.pi/2:
                    self.goal = idx
                    return self.path_points_x[self.goal], self.path_points_y[self.goal], self.path_points_yaw[self.goal]
        
        self.goal = -1 # If no waypoint found, return the last waypoint
        return self.path_points_x[self.goal], self.path_points_y[self.goal], self.path_points_yaw[self.goal]
    
    def get_gem_state(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = self.heading_to_yaw(self.heading) 

        # reference point is located at the center of rear axle
        curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        curr_y = local_y_curr - self.offset * np.sin(curr_yaw)

        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def get_dist_between_car_and_summon_pos(self):        
        local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
        local_x_gps, local_y_gps = self.wps_to_local_xy(self.gps_lon, self.gps_lat)
        return self.dist((local_x_curr, local_y_curr), (local_x_gps, local_y_gps))
    
    def calculate_yaw(self, curr_x, curr_y, target_x, target_y):
        delta_x = target_x - curr_x
        delta_y = target_y - curr_y
        yaw = math.atan2(delta_y, delta_x)  # Yaw in radians
        return yaw

    
    def start_pp(self):
        
        while not rospy.is_shutdown():

            if (self.gem_enable == False):

                if(self.pacmod_enable == True):

                    # ---------- enable PACMod ----------

                    # enable forward gear
                    self.gear_cmd.ui16_cmd = 3

                    # enable brake
                    self.brake_cmd.enable  = True
                    self.brake_cmd.clear   = False
                    self.brake_cmd.ignore  = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable  = True
                    self.accel_cmd.clear   = False
                    self.accel_cmd.ignore  = False
                    self.accel_cmd.f64_cmd = 0.0

                    self.gear_pub.publish(self.gear_cmd)
                    print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    print("Turn Signal Ready!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    print("Gas Engaged!")

                    self.gem_enable = True

            if self.gps_lat != 0 and  self.gps_lon != 0:    
                print("error to gps auto summon point --------------------- ", self.get_dist_between_car_and_summon_pos())

                if self.get_dist_between_car_and_summon_pos() < 5:
                    self.auto_summon_pub.publish(True)
                    self.accel_cmd.f64_cmd = 0
                    self.steer_cmd.angular_position = np.radians(0)
                    self.turn_cmd.ui16_cmd = 1
                    self.brake_cmd.f64_cmd = 1
                    # self.accel_pub.publish(self.accel_cmd)
                    # self.steer_pub.publish(self.steer_cmd)
                    # self.turn_pub.publish(self.turn_cmd)
                    self.turn_pub.publish(self.brake_cmd)
                    self.auto_summon_pub.publish(True)
                    self.gps_lon = 0
                    self.gps_lat = 0 

                curr_x, curr_y, curr_yaw = self.get_gem_state()
                print("current x", curr_x)
                print("current y", curr_y)
                print("current yaw", curr_yaw)
                self.rrtPlotPub.publish(self.rrt_plot_image)

                target_waypoint_x, target_waypoint_y, target_waypoint_yaw = self.find_target_waypoint(curr_x, curr_y, curr_yaw)

                target_waypoint_yaw = self.calculate_yaw(curr_x, curr_y, target_waypoint_x, target_waypoint_y)
                
                self.path_points_x = np.array(self.path_points_x)
                self.path_points_y = np.array(self.path_points_y)


                # finding the distance between the goal point and the vehicle
                # true look-ahead distance between a waypoint and current position
                L =  self.dist((target_waypoint_x, target_waypoint_y), (curr_x, curr_y))
                print("distance to self.goal", L)
                print("x goal:", target_waypoint_x, ", y goal: ", target_waypoint_y, ", yaw goal", target_waypoint_yaw)


                # find the curvature and the angle 
                alpha = target_waypoint_yaw - curr_yaw

                # ----------------- tuning this part as needed -----------------
                k       = 0.41 
                angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L) 
                angle   = angle_i*2
                # ----------------- tuning this part as needed -----------------

                f_delta = round(np.clip(angle, -0.61, 0.61), 3)

                f_delta_deg = np.degrees(f_delta)

                # steering_angle in degrees
                steering_angle = self.front2steer(f_delta_deg)

                if(self.gem_enable == True):
                    print("Current index: " + str(self.goal))
                    print("Forward velocity: " + str(self.speed))
                    ct_error = round(np.sin(alpha) * L, 3)
                    print("Crosstrack Error: " + str(ct_error))
                    print("Front steering angle: " + str(np.degrees(f_delta)) + " degrees")
                    print("Steering wheel angle: " + str(steering_angle) + " degrees" )
                    print("\n")

                current_time = rospy.get_time()
                filt_vel     = self.speed_filter.get_data(self.speed)
                output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)

                if output_accel > self.max_accel:
                    output_accel = self.max_accel

                if output_accel < 0.2:
                    output_accel = 0.2

                if (f_delta_deg <= 30 and f_delta_deg >= -30):
                    self.turn_cmd.ui16_cmd = 1
                elif(f_delta_deg > 30):
                    self.turn_cmd.ui16_cmd = 2 # turn left
                else:
                    self.turn_cmd.ui16_cmd = 0 # turn right

                self.accel_cmd.f64_cmd = output_accel
                self.steer_cmd.angular_position = np.radians(steering_angle)
                self.accel_pub.publish(self.accel_cmd)
                self.steer_pub.publish(self.steer_cmd)
                self.turn_pub.publish(self.turn_cmd)

                self.rate.sleep()


def pure_pursuit():

    rospy.init_node('gnss_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()


