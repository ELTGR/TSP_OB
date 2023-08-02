#!/usr/bin/env python

from __future__ import division


import math

import rospy

import time

import navpy

from testing.bridge import Bridge






try:
    from pubs import Pubs
#     from subs import Subs
#     from video import Video
except:
    from bluerov.pubs import Pubs
#     from bluerov.subs import Subs
#     from bluerov.video import Video

# from TrajectoryGenerator import TrajectoryGenerator

# convert opencv image to ros image msg
from cv_bridge import CvBridge

# msgs type
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
# from bluerov_ros_playground.msg import Bar30
# from bluerov_ros_playground.msg import Attitude 
# from bluerov_ros_playground.msg import State 

"""
Generates a quintic polynomial trajectory.
Author: Daniel Ingram (daniel-s-ingram)
"""

import numpy as np

class TrajectoryGenerator():
    def __init__(self, start_pos, des_pos, T, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0]):
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]

        self.T = T

    def solve(self):
        A = np.array(
            [[0, 0, 0, 0, 0, 1],
             [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
             [0, 0, 0, 0, 1, 0],
             [5*self.T**4, 4*self.T**3, 3*self.T**2, 2*self.T, 1, 0],
             [0, 0, 0, 2, 0, 0],
             [20*self.T**3, 12*self.T**2, 6*self.T, 2, 0, 0]
            ])

        b_x = np.array(
            [[self.start_x],
             [self.des_x],
             [self.start_x_vel],
             [self.des_x_vel],
             [self.start_x_acc],
             [self.des_x_acc]
            ])
        
        b_y = np.array(
            [[self.start_y],
             [self.des_y],
             [self.start_y_vel],
             [self.des_y_vel],
             [self.start_y_acc],
             [self.des_y_acc]
            ])

        b_z = np.array(
            [[self.start_z],
             [self.des_z],
             [self.start_z_vel],
             [self.des_z_vel],
             [self.start_z_acc],
             [self.des_z_acc]
            ])

        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)


class BlueRov(Bridge):
    
    def __init__(self, device='udp:192.168.2.1:14550', baudrate=115200):
        """ BlueRov ROS Bridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate)
        self.pub = Pubs()
        # self.sub = Subs()
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'

        # # self.video = Video()
        # self.video_bridge = CvBridge()

        self.pub_topics = [
            [
                self._create_position_msg,
                '/local_position',
                PoseStamped,
                1
            ],
            
            [
                self._create_odometry_msg,
                '/odometry',
                Odometry,
                1
            ]
        ]

        self.mavlink_msg_available = {}

        for _, topic, msg, queue in self.pub_topics:
            self.mavlink_msg_available[topic] = 0
            self._pub_subscribe_topic(topic, msg, queue)

    @staticmethod

    def pub_pass(self):
        pass
   
    def _pub_subscribe_topic(self, topic, msg, queue_size=1):
     
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

    def gps2ned(point_gps_x, point_gps_y) :

        #prend le premier X Y pour points de référence ned 0,0 pour calculer les autres point 
        lat_ref=point_gps_x[0]
        lon_ref=point_gps_y[0]
        alt_ref=0


        #¢reation des listes qui vont recevoire les coordonnées
        point_ned_x=[]
        point_ned_y=[]

        #calculer de transformation de GPS a NED puis ajouter dans les listes 
        for i in range(1,len(point_gps_x)) : 

            x,y,z=navpy.lla2ned(point_gps_x[i],point_gps_y[i],0 ,lat_ref , lon_ref,alt_ref, latlon_unit='deg', alt_unit='m', model='wgs84')
            point_ned_x.append(x)
            point_ned_y.append(y)
            

        #retour des points NED
        return point_ned_x,point_ned_y
    
    def _create_header(self, msg):
       
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    # TODO : tester l'utilisation des vélocités comme dans _create_odometry_msg

    def _create_position_msg(self):
     
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = PoseStamped()

        self._create_header(msg)

        # http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.position.x = xyz_data[0]
        msg.pose.position.y = xyz_data[1]
        msg.pose.position.z = - xyz_data[2]
        # print(xyz_data)

        # https://mavlink.io/en/messages/common.html#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion

        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)


        msg.pose.orientation.w = 1
        msg.pose.orientation.x = math.degrees(orientation[0])
        msg.pose.orientation.y = math.degrees(orientation[1])
        msg.pose.orientation.z = math.degrees(orientation[2])
        
        self.pub.set_data('/local_position', msg)

    def _create_odometry_msg(self):
        
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = Odometry()

        self._create_header(msg)

        #http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.pose.position.x = xyz_data[0]
        msg.pose.pose.position.y = xyz_data[1]
        msg.pose.pose.position.z = xyz_data[2]
        msg.twist.twist.linear.x = vxyz_data[0]/100
        msg.twist.twist.linear.y = vxyz_data[1]/100
        msg.twist.twist.linear.z = vxyz_data[2]/100

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = orientation_speed[1]
        msg.twist.twist.angular.z = orientation_speed[2]

        self.pub.set_data('/odometry', msg)

    def get_battery_percentage(self):
        #récupère simplement le poucentage de la batterie
        # bat_percentage = self.get_data()['BATTERY_STATUS']['battery_remaining']/100
        bat_percentage = 100
        return bat_percentage
    
    def get_current_pose(self) : 
        '''
            Returns current NED position of the bluerov with numpy array type : array([x, y, z]) 
        '''
        self.get_bluerov_data()
        current_pose = np.array(self.current_pose)
        return current_pose

    def publish(self):
        """ Publish the data in ROS topics
        """
        self.update()
        for sender, topic, _, _ in self.pub_topics:
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    sender()
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)
   
    def do_recalibrage(self, current_position):
        '''
            Make the bluerov get reach the surface in order to recalibrate his position with gps, then 
            get back to his working depth
        '''
        self.get_bluerov_data()
        self.mission_ongoing = True
        rate = rospy.Rate(50.0)
        desired_position = [current_position[0], current_position[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.set_position_target_local_ned(desired_position)
        while self.current_pose[2] < -0.1:
            self.get_bluerov_data()
            self.publish()
            rate.sleep()
        time.sleep(2)
        desired_position = [current_position[0], current_position[1], -current_position[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.set_position_target_local_ned(desired_position)
        while self.current_pose[2] > current_position[2]:
            self.get_bluerov_data()
            self.publish()
            rate.sleep()
        self.mission_ongoing = False



