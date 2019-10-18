#!/usr/bin/python

import os
import time
from system_manager import SystemManager

import rospy
import rosnode
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float64
from gps_common.msg import GPSFix
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from state_observer.srv import SetMode
from state_observer.msg import Diagnostics
from geometry_msgs.msg import Quaternion, Twist, PoseStamped

NODES = {

    'imu': {'name': 'imu', 'topic': '/imu', 'script': 'start_imu.sh', 'status': 'stopped', 'topic_type': Imu, 'timeout': 15.0 },
    'drive': {'name': 'drive', 'topic': '/odom_wheel', 'script': 'start_drive.sh', 'status': 'stopped', 'topic_type': Odometry, 'timeout': 5.0 },
    'lidar': {'name': 'lidar', 'topic': '/scan', 'tag': '/LIDAR', 'script': 'start_lidar.sh', 'status': 'stopped', 'topic_type': LaserScan, 'timeout': 5.0 },
    'realsense': {'name': 'realsense', 'topic': '/camera/depth/color/points', 'script': 'start_realsense.sh', 'status': 'stopped', 'topic_type': PointCloud2, 'timeout': 10.0},
    'icp':{'name': 'icp', 'topic': '/odom_lidar', 'script': 'start_lidar_icp.sh', 'status': 'stopped', 'topic_type': Odometry, 'timeout': 5.0 },
    'ekf_inertial': {'name': 'ekf_inertial', 'topic': '/odom_inertial', 'script': 'start_ekf_inertial.sh', 'status': 'stopped', 'topic_type': Odometry, 'timeout': 5.0 },
    'map_inertial': {'name': 'map_inertial', 'topic': '/map', 'script': 'start_map_inertial.sh', 'status': 'stopped', 'topic_type': OccupancyGrid, 'timeout': 10.0 },
    'nav_inertial': {'name':'nav_inertial', 'topic':'/MOVE_INERTIAL/local_costmap/costmap', 'script':'start_nav_inertial.sh', 'status': 'stopped', 'topic_type': OccupancyGrid, 'timeout': 5.0 },
    'gps_driver': {'name': 'gps_driver', 'topic': '/gps_fix', 'script': 'start_gps_driver.sh', 'status': 'stopped', 'topic_type': GPSFix, 'timeout': 5.0 },
    'gps_conv': {'name': 'gps_conv', 'topic': '/gps_navsat', 'script': 'start_gps_converter.sh', 'status': 'stopped', 'topic_type': NavSatFix, 'timeout': 5.0},
    'gps_init': {'name': 'gps_init', 'topic': '/local_xy_origin', 'script': 'start_gps_initializer.sh', 'status': 'stopped', 'topic_type': PoseStamped, 'timeout': 5.0},
    'nav_sat': {'name': 'nav_sat', 'topic': '/odom_gps', 'script': 'start_nav_sat.sh', 'status': 'stopped', 'topic_type': Odometry, 'timeout': 5.0 },
    'ekf_global': {'name': 'ekf_global', 'topic': '/odom_global', 'script': 'start_ekf_global.sh', 'status': 'stopped', 'topic_type': Odometry, 'timeout': 5.0 },
    'nav_global': {'name': 'nav_global', 'topic': '/MOVE_GLOBAL/local_costmap/costmap', 'script': 'start_nav_global.sh', 'status': 'stopped', 'topic_type': OccupancyGrid, 'timeout': 5.0 },
    'map_global': {'name': 'map_global', 'topic': '/map_global', 'script': 'start_map_global.sh', 'status': 'stopped', 'topic_type': OccupancyGrid, 'timeout': 5.0 },
    'control_global': {'name': 'control_global', 'topic': '/controller_check', 'script': 'start_control_global.sh', 'status': 'stopped', 'topic_type': Float64, 'timeout': 5.0 },
    'avoid_inertial': {'name': 'avoid_inertial', 'topic': '/obstacles', 'script': 'start_avoidance_inertial.sh', 'status': 'stopped', 'topic_type': PointCloud2, 'timeout': 5.0 },
    'avoid_global': {'name': 'avoid_global', 'topic': '/obstacles', 'script': 'start_avoidance_global.sh', 'status': 'stopped', 'topic_type': PointCloud2, 'timeout': 5.0 }
}

class Observer:

    def __init__(self):

        self.manager = SystemManager()

        self.common_nodes = {k:v for k,v in NODES.items() if k in ['imu', 'drive', 'icp', 'ekf_inertial', 'lidar', 'realsense']}.values()

        self.inertial_nodes = {k:v for k,v in NODES.items() if k in ['map_inertial', 'nav_inertial', 'avoid_inertial']}.values()

        self.global_nodes = {k:v for k,v in NODES.items() if k in ['gps_driver','gps_conv', 'nav_sat', 'ekf_global', 'nav_global', 'control_global', 'avoid_global']}.values()

        self.system_states = ['idle', 'broadcasting', 'fault']

        self.system_modes = ['', 'inertial', 'global']

        self.system_nodes = {'': [], 'inertial': self.inertial_nodes, 'global': self.global_nodes}

        self.current_system_mode = ''

        self.current_system_diagnostics = ''

        self.set_mode_on = False

        self.update_system_on = False

        self.failed_nodes = []

        self.to_be_healed = []


    def update_system_info(self, which_nodes = 'all'):

        if which_nodes == 'all':

            current_nodes = self.common_nodes + self.system_nodes[self.current_system_mode]

        elif which_nodes == 'healed':

            current_nodes = self.to_be_healed

        self.failed_nodes = []

        # checking node health
        for node in current_nodes:

            try:

                rospy.wait_for_message(node['topic'], node['topic_type'], node['timeout'])

            except:

                self.failed_nodes.append(node['name'])


    def heal_nodes(self):

        self.to_be_healed = {k:v for k,v in NODES.items() if k in self.failed_nodes}.values()

        self.manager.restart_stack(self.to_be_healed)


    def get_system_info(self):

        if self.set_mode_on:

            self.current_system_diagnostics = 'changing mode'

        else:

            self.update_system_on = True

            self.update_system_info()

            print('Initial test: ')

            print(self.failed_nodes)

            if self.failed_nodes != []:

                self.heal_nodes()

                self.update_system_info(which_nodes='healed')

                print('After healing: ')
                
                print(self.failed_nodes)

                if self.failed_nodes != []:

                    self.current_system_diagnostics = 'self-healing attempted but failed. faulty nodes: ' + str(self.failed_nodes)

                else:

                    self.current_system_diagnostics = 'self-healing completed successfully. system healthy'

            else:

                self.current_system_diagnostics = 'system healthy'
                
            self.update_system_on = False

        return (self.current_system_mode, self.current_system_diagnostics)


    def set_system_mode(self, new_mode):

        # wait for the system to complete updating its status
        while self.update_system_on:

            time.sleep(0.25)

        # now, blocking updates until new mode is set
        self.set_mode_on = True

        self.manager.stop_stack(self.system_nodes[self.current_system_mode])

        self.current_system_mode = new_mode

        self.manager.start_stack(self.system_nodes[new_mode])

        self.set_mode_on = False


    def system_reset(self):

        self.manager.restart_stack(self.common_nodes)

        if self.current_system_mode == 'inertial':

            self.manager.restart_stack(self.inertial_nodes)

        else:

            self.manager.restart_stack(self.global_nodes)

            

    
           
