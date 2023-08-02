#!/usr/bin/env python

from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import math

import numpy as np
# from .. import attitude_control



from sensor_msgs.msg import LaserScan, PointCloud2, PointCloud
import sensor_msgs.point_cloud2 as pc2
import rospy

class Bridge(object):
   
    def __init__(self, device='udpin:192.168.2.1:14560', baudrate=115200):
      
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        
        self.conn.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.conn.target_system, self.conn.target_component))
            
        self.data = {}
        self.update()
        # [x, y, z] selon EKF
        self.current_pose = [0,0,0]
        # [roll, pitch, yaw] en radians
        self.current_attitude = [0,0,0]
        # [vx, vy, vz] en m/s
        self.current_vel = [0,0,0]
        self.ob = np.array([[]])
        self.mission_scan_point = 0
        self.x_evit = np.array([])
        self.scan = LaserScan()
        self.obstacles_colliders = PointCloud()

        self.evitement = None
        self.mission_point_sent = False
        self.init_evit = False
        self.ok_pose = False
        self.mission_ongoing = False
        self.mission_evit = False
        self.mission_scan = False

        self.scan_subscriber= rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.velodyne_subscriber= rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback, queue_size=1)
        # self.collider_subscriber= rospy.Subscriber("/BlueRov2/obstacles", PointCloud, self.collider_callback, queue_size=100)

        # Dimension observations velodyne
        self.environment_dim = 20
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        self.vfh_sector_angle = 1.0
        self.vfh_sector_angles = np.linspace(-45 + 90/(90*2), 45 + 90/(90*2), 90)
        self.vfh_sector_range = 5.0


    def velodyne_callback(self, v):
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(self.environment_dim) * 10
        for i in range(len(data)):
            if data[i][2] > -0.2:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                        break

    def scan_callback(self, data):
        self.scan = data

    def collider_callback(self, data):
        self.obstacles_colliders = data

    #################################### Fonctions commandes MAVLINK ###############################

    def get_data(self):
        """ Return data

        Returns:
            TYPE: Dict
        """
        return self.data

    def get_all_msgs(self):
        """ Return all mavlink messages

        Returns:
            TYPE: dict
        """
        msgs = []
        while True:
            msg = self.conn.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def update(self):
        """ Update data dict
        """
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    def print_data(self):
        """ Debug function, print data dict
        """
        print(self.data)

    def set_mode(self, mode):
        """ Set ROV mode
            http://ardupilot.org/copter/docs/flight-modes.html

        Args:
            mode (str): MMAVLink mode

        Returns:
            TYPE: Description
        """
        mode = mode.upper()
        if mode not in self.conn.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.conn.mode_mapping().keys()))
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)

    def decode_mode(self, base_mode, custom_mode):
       
        flight_mode = ""

        mode_list = [
            [mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 'MANUAL'],
            [mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED, 'STABILIZE'],
            [mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED, 'GUIDED'],
            [mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED, 'AUTO'],
            [mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED, 'TEST']
        ]

        if base_mode == 0:
            flight_mode = "PreFlight"
        elif base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
            flight_mode = mavutil.mode_mapping_sub[custom_mode]
        else:
            for mode_value, mode_name in mode_list:
                if base_mode & mode_value:
                    flight_mode = mode_name

        arm = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        return flight_mode, arm

    def set_guided_mode(self):
        """ Set guided mode
        """
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)

    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        """ Function to abstract long commands

        Args:
            command (mavlink command): Command
            params (list, optional): param1, param2, ..., param7
            confirmation (int, optional): Confirmation value
        """
        self.conn.mav.command_long_send(
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            command,                                # mavlink command
            confirmation,                           # confirmation
            params[0],                              # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6]
        )

    def set_position_target_local_ned(self, param=[]):
        
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')


        while not self.is_armed():
            self.conn.arducopter_arm()

        self.conn.set_mode('GUIDED')
        mask = 0b10011111000

        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        self.conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, self.conn.target_system, self.conn.target_component,
                           mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(mask),
                            param[0], param[1], param[2],                   # position x,y,z
                            param[3], param[4], param[5],                   # velocity x,y,z
                            param[6], param[7], param[8],                   # accel x,y,z
                            param[9], param[10]))                           # yaw, yaw rate


    def set_attitude_target(self, param=[]):
        
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')

        # Set mask
        mask = 0b11111111
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask -= 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.conn.mav.set_attitude_target_send(0,   # system time in milliseconds
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            mask,                                   # mask
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust

    def set_servo_pwm(self, id, pwm=1500):
       
        mavutil.mavfile.set_servo(self.conn, id, pwm)

    def set_rc_channel_pwm(self, id, pwm=1500):
        
        rc_channel_values = [65535 for _ in range(8)] #8 for mavlink1
        rc_channel_values[id] = pwm
        #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,                # target_system
            self.conn.target_component,             # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.
    
    def set_manual_control(self,joy_list=[0]*4, buttons_list=[0]*16):
        
        x,y,z,r = 0,0,0,0#32767,32767,32767,32767
        b = 0
        for i in range(len(buttons_list)):
            b = b | (buttons_list[i]<<i)
        print("MANUAL_CONTROL_SEND : x : {}, y : {}, z : {}, r : {}, b : {}".format(x,y,z,r,b))
        #https://mavlink.io/en/messages/common.html MANUAL_CONTROL ( #69 )
        self.conn.mav.manual_control_send(
               self.conn.target_system,
                x,
                y,
                z,
                r,
                b)


    def arm_throttle(self, arm_throttle):
        
        if arm_throttle:
            self.conn.arducopter_arm()
        else:
            #http://mavlink.org/messages/common#MAV_CMD_COMPONENT_ARM_DISARM
            # param1 (0 to indicate disarm)
            # Reserved (all remaining params)
            self.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0, 0, 0, 0, 0, 0, 0]
            )



    def set_target_depth(self,depth):

        self.conn.set_mode('ALT_HOLD')

        while not self.is_armed():
            self.conn.arducopter_arm()

        print('Bluerov is armed')

        self.conn.mav.set_position_target_global_int_send(
            0,     
            0, 0,   
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT, # frame
            0b0000111111111000,
            0,0, depth,
            0 , 0 , 0 , # x , y , z velocity in m/ s ( not used )
            0 , 0 , 0 , # x , y , z acceleration ( not supported yet , ignored in GCS Mavlink )
            0 , 0 ) # yaw , yawrate ( not supported yet , ignored in GCS Mavlink )

        print('set_position_target_global_int_send')    

    def is_armed(self):
        try:
            return bool(self.conn.wait_heartbeat().base_mode & 0b10000000)
        except:
            return False  


    def set_target_attitude(self, roll, pitch, yaw, control_yaw=True):
        bitmask = (1<<6 | 1<<3)  if control_yaw else 1<<6

        self.conn.mav.set_attitude_target_send(
            0,     
            0, 0,   
            bitmask,
            QuaternionBase([math.radians(roll), math.radians(pitch), math.radians(yaw)]), # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            0, #roll rate
            0, #pitch rate
            0, 0)    # yaw rate, thrust 

    def get_bluerov_data(self): #Loop comunicazione con QGROUND
        # Get some information !
        self.update()
        if 'LOCAL_POSITION_NED' in self.get_data():              
            
            local_position_data = self.get_data()['LOCAL_POSITION_NED']
            xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
            self.current_pose = [xyz_data[0], xyz_data[1], xyz_data[2]]
            # print(xyz_data)
            vxvyvz_data = [local_position_data[i]  for i in ['vx', 'vy', 'vz']]
            self.current_vel = [vxvyvz_data[0], vxvyvz_data[1], vxvyvz_data[2]]
            attitude_data = self.get_data()['ATTITUDE']
            orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
            self.current_attitude = [orientation[0], orientation[1], orientation[2]]

    ###################################### Fin fonctions commandes MAVLINK #############################################

   

    ################################## Fonctions de détection d'obstacles ###########################################
    def get_collider_obstacles(self):
        obstacles = np.array([[p.x, p.z, p.y] for p in self.obstacles_colliders.points])
        # print(obstacles)
        return obstacles


    def get_velodyne_obstacle(self):

        inclinaison_lidar = 0
        obstacle =  np.zeros((self.environment_dim,2))
        velodyne_data = np.flip(self.velodyne_data, 0)

        depart = -45 + 90/(self.environment_dim*2)
        arrivee = 45 - 90/(self.environment_dim*2)

        scan_zone_angles = np.linspace(depart, arrivee, self.environment_dim)

        for i in range(self.environment_dim):
            if velodyne_data[i] == 10:
                obstacle[i, 0], obstacle[i, 1] = float("Inf"), float("Inf")
            else :
                obstacle_min_range = round(velodyne_data[i], 2)
                obstacle_angle = math.radians(scan_zone_angles[i])
                # print(scan_zone_angles[scan_zone_flag-1])
                # on calcule tout d'abord les coordonnées de l'objet dans le repère relatif au robot
                obstacle_rel_x = obstacle_min_range * math.cos(obstacle_angle)
                obstacle_rel_y = obstacle_min_range * math.sin(obstacle_angle)
                obstacle_rel_z = obstacle_min_range * math.sin(inclinaison_lidar)
                # On définit ensuite la matrice de transformation homogène H du robot à partir de ses matrices de transposition T et rotation R

                T = np.array([[1, 0, 0, self.current_pose[0]],
                                [0, 1, 0, self.current_pose[1]], 
                                [0, 0, 1, self.current_pose[2]],
                                [0, 0, 0, 1]])
                
                R = np.array([[math.cos(self.current_attitude[1])*math.cos(self.current_attitude[2]), -math.cos(self.current_attitude[1])*math.sin(self.current_attitude[2]), math.sin(self.current_attitude[1]), 0],
                                [math.sin(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.cos(self.current_attitude[2])+math.cos(self.current_attitude[0])*math.sin(self.current_attitude[2]), -math.sin(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.sin(self.current_attitude[2])+math.cos(self.current_attitude[0])*math.cos(self.current_attitude[2]), -math.sin(self.current_attitude[0])*math.cos(self.current_attitude[1]), 0],
                                [-math.cos(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.cos(self.current_attitude[2])+math.sin(self.current_attitude[0])*math.sin(self.current_attitude[2]), math.cos(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.sin(self.current_attitude[2])+math.sin(self.current_attitude[0])*math.cos(self.current_attitude[2]), math.cos(self.current_attitude[0])*math.cos(self.current_attitude[1]), 0],
                                [0, 0, 0, 1]])
                
                H = np.dot(T,R)

                # On peut maintenant calculer la matrice de coordonnées de l'obstacle 
                coord_obj_abs = np.dot(np.array([[obstacle_rel_x, obstacle_rel_y, obstacle_rel_z, 1]]), H)
                obstacle[i, 0], obstacle[i, 1] = coord_obj_abs[0,0] + self.current_pose[0], coord_obj_abs[0,1] + self.current_pose[1]
        return obstacle

    def get_lidar_obstacle(self, decoupage):
       
        inclinaison_lidar = 0
        obstacle =  np.zeros((decoupage,2))
        scan_increments = len(self.scan.ranges)

        depart = -45 + 90/(decoupage*2)
        arrivee = 45 - 90/(decoupage*2)

        scan_zone_angles = np.linspace(depart, arrivee, decoupage)
        scan_zone_flag = 1
        min_range = []

        for i in range(scan_increments):
            # on parcourt tous les lasers du lidar
            # on ajoute les valeurs non nulles à la liste min_range
            if self.scan.ranges[i] != 0:
                min_range.append(self.scan.ranges[i])

            if i == scan_zone_flag * (scan_increments / decoupage) -1:
                # On a rassemblé toutes les valeurs d'une zone
                if len(min_range) == 0:
                    # Si toutes les valeurs de la zone étaient nulles (le lidar n'a rien détecté), alors on renvoie la valeur Inf
                    obstacle[scan_zone_flag - 1, 0], obstacle[scan_zone_flag - 1, 1] = float("Inf"), float("Inf")
                else:
                    # Sinon on calcule les coordonnées de l'objet détecté dans la zone 
                    obstacle_min_range = round(min(min_range), 2)
                    obstacle_angle = math.radians(scan_zone_angles[scan_zone_flag-1])
                    # print(scan_zone_angles[scan_zone_flag-1])
                    # on calcule tout d'abord les coordonnées de l'objet dans le repère relatif au robot
                    obstacle_rel_x = obstacle_min_range * math.cos(obstacle_angle)
                    obstacle_rel_y = obstacle_min_range * math.sin(obstacle_angle)
                    obstacle_rel_z = obstacle_min_range * math.sin(inclinaison_lidar)
                    # On définit ensuite la matrice de transformation homogène H du robot à partir de ses matrices de transposition T et rotation R

                    T = np.array([[1, 0, 0, self.current_pose[0]],
                                   [0, 1, 0, self.current_pose[1]], 
                                   [0, 0, 1, self.current_pose[2]],
                                   [0, 0, 0, 1]])
                    
                    R = np.array([[math.cos(self.current_attitude[1])*math.cos(self.current_attitude[2]), -math.cos(self.current_attitude[1])*math.sin(self.current_attitude[2]), math.sin(self.current_attitude[1]), 0],
                                  [math.sin(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.cos(self.current_attitude[2])+math.cos(self.current_attitude[0])*math.sin(self.current_attitude[2]), -math.sin(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.sin(self.current_attitude[2])+math.cos(self.current_attitude[0])*math.cos(self.current_attitude[2]), -math.sin(self.current_attitude[0])*math.cos(self.current_attitude[1]), 0],
                                  [-math.cos(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.cos(self.current_attitude[2])+math.sin(self.current_attitude[0])*math.sin(self.current_attitude[2]), math.cos(self.current_attitude[0])*math.sin(self.current_attitude[1])*math.sin(self.current_attitude[2])+math.sin(self.current_attitude[0])*math.cos(self.current_attitude[2]), math.cos(self.current_attitude[0])*math.cos(self.current_attitude[1]), 0],
                                  [0, 0, 0, 1]])
                    
                    H = np.dot(T,R)

                    # On peut maintenant calculer la matrice de coordonnées de l'obstacle 
                    coord_obj_abs = np.dot(np.array([[obstacle_rel_x, obstacle_rel_y, obstacle_rel_z, 1]]), H)
                    obstacle[scan_zone_flag - 1, 0], obstacle[scan_zone_flag - 1, 1] = coord_obj_abs[0,0] + self.current_pose[0], coord_obj_abs[0,1] + self.current_pose[1]
                min_range = []
                scan_zone_flag += 1

                # R = | cos(Pitch)*cos(Yaw) -cos(Pitch)*sin(Yaw) sin(Pitch) 0 |
                #     | sin(Roll)*sin(Pitch)*cos(Yaw)+cos(Roll)*sin(Yaw) -sin(Roll)*sin(Pitch)*sin(Yaw)+cos(Roll)*cos(Yaw) -sin(Roll)*cos(Pitch) 0 |
                #     | -cos(Roll)*sin(Pitch)*cos(Yaw)+sin(Roll)*sin(Yaw) cos(Roll)*sin(Pitch)*sin(Yaw)+sin(Roll)*cos(Yaw) cos(Roll)*cos(Pitch) 0 |
                #     | 0 0 0 1 |   

        return obstacle
    ############################################################ Fin fonction détection d'obstacles ##############################################
    ############################################################ Fonctions Utiles ################################################################
    def scan_zone(self, start, goal):
        '''
            Method to define the vertices of the square the robot has to scan.
        '''
        # we suppose square is 20x20. The conditions depend on the scenario we made only.
        print(start)
        print(goal)
        if start[1] % 20 == 0 and start[0] % 20 != 0:
            if start[0] > goal[0]:
                ox = [start[0], start[0], start[0]-20, start[0]-20, start[0]]
                oy = [start[1]+10, start[1]-10, start[1]-10, start[1]+10, start[1]+10]
            else:
                ox = [start[0], start[0], start[0]+20, start[0]+20, start[0]]
                oy = [start[1]+10, start[1]-10, start[1]-10, start[1]+10, start[1]+10]
        else:
            if start[1] > goal[1]:
                ox = [start[0]+10, start[0]-10, start[0]-10, start[0]+10, start[0]+10]
                oy = [start[1], start[1], start[1]-20, start[1]-20, start[1]]
            else:
                ox = [start[0]+10, start[0]-10, start[0]-10, start[0]+10, start[0]+10]
                oy = [start[1], start[1], start[1]+20, start[1]+20, start[1]]

        return ox, oy
        
################################################################ Fin classe Bridge ###############################################################

if __name__ == '__main__':
    bridge = Bridge()
    
    while True:
        bridge.update()
        bridge.print_data()
        