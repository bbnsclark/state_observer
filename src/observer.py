#!/usr/bin/python

import os
import sys
import yaml
import time
from websocket import create_connection
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

# NODES = {

#     'sitl': {'topic': '/odom_wheel', 'script': 'start_gazebo_sitl.sh', 'method': 'topic', 'topic_type': Odometry, 'timeout': 5.0 },
#     'firmware': {'topic': '/odom_wheel', 'script': 'start_sitl_firmware.sh', 'method': 'node', 'topic_type': Odometry, 'timeout': 5.0 },
#     'nav_sat': {'topic': '/odom_gps', 'script': 'start_nav_sat.sh', 'method': 'topic', 'topic_type': Odometry, 'timeout': 5.0 },
#     'icp':{'topic': '/odom_lidar', 'script': 'start_lidar_icp.sh', 'method': 'topic', 'topic_type': Odometry, 'timeout': 5.0 },
#     'ekf_inertial': {'topic': '/odom_inertial', 'script': 'start_ekf_inertial.sh', 'method': 'topic', 'topic_type': Odometry, 'timeout': 5.0 },
#     'map_inertial': {'topic': '/map', 'script': 'start_map_inertial.sh', 'method': 'topic', 'topic_type': OccupancyGrid, 'timeout': 2.0 },
#     'map_local': {'topic': '/map_pose', 'script': 'start_map_localization.sh', 'method': 'topic', 'topic_type': PoseStamped, 'timeout': 2.0 },
#     'nav_inertial': {'topic':'/MOVE_INERTIAL/local_costmap/costmap', 'script':'start_nav_inertial.sh', 'method': 'topic', 'topic_type': OccupancyGrid, 'timeout': 5.0 },
#     'gps_driver_gazebo': {'topic': '/gps_fix', 'script': 'start_gps_driver_gazebo.sh', 'method': 'topic', 'topic_type': GPSFix, 'timeout': 5.0 },
#     'gps_driver_airsim': {'topic': '/gps_fix', 'script': 'start_gps_driver_airsim.sh', 'method': 'topic', 'topic_type': GPSFix, 'timeout': 5.0 },
#     'gps_conv': {'topic': '/gps_navsat', 'script': 'start_gps_converter.sh', 'method': 'topic', 'topic_type': NavSatFix, 'timeout': 5.0},
#     'gps_init': {'topic': '/local_xy_origin', 'script': 'start_gps_initializer.sh', 'method': 'topic', 'topic_type': PoseStamped, 'timeout': 5.0},
#     'ekf_global': {'topic': '/odom_global', 'script': 'start_ekf_global.sh', 'method': 'topic', 'topic_type': Odometry, 'timeout': 5.0 },
#     'nav_global': {'topic': '/MOVE_GLOBAL/local_costmap/costmap', 'script': 'start_nav_global.sh', 'method': 'topic', 'topic_type': OccupancyGrid, 'timeout': 5.0 },
#     'map_global': {'topic': '/map_global', 'script': 'start_map_global.sh', 'method': 'topic', 'topic_type': OccupancyGrid, 'timeout': 5.0 },
#     'control_global': {'topic': '/controller_check', 'script': 'start_control_global.sh', 'method': 'topic', 'topic_type': Float64, 'timeout': 5.0 },
#     'avoid_inertial': {'topic': '/obstacles', 'script': 'start_avoidance_inertial.sh', 'method': 'node', 'topic_type': PointCloud2, 'timeout': 5.0 },
#     'avoid_global': {'topic': '/obstacles', 'script': 'start_avoidance_global.sh', 'method': 'node', 'topic_type': PointCloud2, 'timeout': 5.0 },
#     'nav_trans': {'topic': '/MOVE_TRANS/local_costmap/costmap', 'script': 'start_nav_trans.sh', 'method': 'topic', 'topic_type': OccupancyGrid, 'timeout': 5.0 },
#     'rviz_inertial': {'topic': '', 'script': 'start_rviz_inertial.sh', 'method': 'none', 'topic_type': None, 'timeout': 5.0 },
#     'rviz_trans': {'topic': '', 'script': 'start_rviz_transition.sh', 'method': 'none', 'topic_type': None, 'timeout': 5.0 },
#     'rviz_global': {'topic': '', 'script': 'start_rviz_global.sh', 'method': 'none', 'topic_type': None, 'timeout': 5.0 },
#     'explore': {'topic': '', 'script': 'start_exploration_server.sh', 'method': 'node', 'topic_type': None, 'timeout' : 5.0 },
#     'rosbridge': {'name': 'rosbridge', 'topic': 9090, 'script': 'start_rosbridge.sh', 'method': 'websocket', 'topic_type': 'port', 'timeout' : 5.0 }
    
# }

class Observer:

    def __init__(self):

        self.manager = SystemManager()
            
        is_sitl = rospy.get_param('~is_sitl', "False")
            
        is_airsim = rospy.get_param('~is_airsim', "False")

        if is_sitl:

            filename = 'states_sitl.yaml'

        else:

            filename = 'states.yaml'
        
        with open(sys.path[0] + '/../config/' + filename, 'r') as stream:

            NODES = yaml.load(stream)

        for k,v in NODES.items():
                    
            v['name'] = k

            v['topic_type'] = eval(v['topic_type'])

        if is_airsim:

            print("AirSim mode")

            self.common_nodes = {k:v for k,v in NODES.items() if k in ['firmware', 'icp', 'ekf_inertial']}.values()

            self.global_nodes = {k:v for k,v in NODES.items() if k in ['gps_driver_airsim', 'nav_sat', 'ekf_global', 'nav_global', 'control_global', 'avoid_global', 'rviz_global']}.values()
        
        else:

            print("Gazebo mode")

            self.common_nodes = {k:v for k,v in NODES.items() if k in ['sitl', 'firmware', 'icp', 'ekf_inertial']}.values()

            self.global_nodes = {k:v for k,v in NODES.items() if k in ['gps_driver_gazebo','gps_conv', 'nav_sat', 'ekf_global', 'nav_global', 'control_global', 'avoid_global', 'rviz_global']}.values()

        self.transition_nodes = {k:v for k,v in NODES.items() if k in ['nav_trans', 'avoid_inertial', 'rviz_trans']}.values()

        self.inertial_nodes = {k:v for k,v in NODES.items() if k in ['map_inertial', 'nav_inertial', 'map_local', 'avoid_inertial', 'rviz_inertial', 'explore']}.values()

        self.system_states = ['idle', 'broadcasting', 'fault']

        self.system_modes = ['', 'inertial', 'global', 'transition']

        self.system_nodes = {'': [], 'inertial': self.inertial_nodes, 'global': self.global_nodes, 'transition': self.transition_nodes}

        self.current_system_mode = ''

        self.current_system_diagnostics = ''

        self.startup_mode = True

        self.update_system_on = False

        self.failed_nodes = []

        self.to_be_healed = []

        self.count = 0


    def update_system_info(self, which_nodes = 'all'):

        self.failed_nodes = []

        if which_nodes == 'all':

            current_nodes = self.common_nodes + self.system_nodes[self.current_system_mode]

        elif which_nodes == 'healed':

            current_nodes = self.to_be_healed

        # checking node health
        for node in current_nodes:

            if node['method'] == 'topic':

                try:

                    rospy.wait_for_message(node['topic'], node['topic_type'], node['timeout'])

                except:

                    self.failed_nodes.append(node['name'])

            elif node['method'] == 'node':

                if self.manager.check_package(node['name']):

                    self.failed_nodes.append(node['name'])

            elif node['method'] == 'websocket':

                try: 

                    self.ws = create_connection("ws://localhost:9090")

                    self.ws.send("ping_websocket")

                except:

                    self.failed_nodes.append(node['name'])


    def heal_nodes(self):

        self.to_be_healed = {k:v for k,v in NODES.items() if k in self.failed_nodes}.values()

        print(self.to_be_healed)

        self.manager.restart_stack(self.to_be_healed)


    def get_system_info(self):

        self.update_system_on = True

        if self.failed_nodes != []: 

            self.current_system_diagnostics = 'faulty nodes: ' + str(self.failed_nodes)

            self.heal_nodes()

        else:

            self.current_system_diagnostics = 'system healthy'

        self.startup_mode = False

        self.update_system_on = False

        time.sleep(1.0)
        
        return (self.current_system_mode, self.current_system_diagnostics)


    def set_system_mode(self, new_mode):

        while self.update_system_on:

            time.sleep(0.5)

        to_be_stopped = [x for x in self.system_modes if x != new_mode]

        for stack in to_be_stopped:

            self.manager.stop_stack(self.system_nodes[stack])

        self.current_system_mode = new_mode

        self.manager.restart_stack(self.system_nodes[new_mode])

        return 'mode set to: ' + str(self.current_system_mode)


    def system_reset(self):

        self.count = 0
        
        self.startup_mode = True

        self.manager.restart_stack(self.common_nodes)

        if self.current_system_mode == 'inertial':

            self.manager.restart_stack(self.inertial_nodes)

        else:

            self.manager.restart_stack(self.global_nodes)

            

    
           
