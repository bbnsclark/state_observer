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

class Observer:

    def __init__(self):

        self.manager = SystemManager()
            
        is_sitl = rospy.get_param('~is_sitl', 'False')
            
        is_airsim = rospy.get_param('~is_airsim', 'False')

        if is_sitl:

            nodes_filename = 'nodes_sitl.yaml'

        else:

            nodes_filename = 'nodes.yaml'
        
        with open(sys.path[0] + '/../config/' + nodes_filename, 'r') as stream:

            NODES = yaml.load(stream)

        for k,v in NODES.items():
                    
            v['name'] = k

            v['topic_type'] = eval(v['topic_type'])

        if is_airsim == True:

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

        self.update_system_info()

        if self.failed_nodes != []: 

            self.current_system_diagnostics = 'faulty nodes: ' + str(self.failed_nodes)

        else:

            self.current_system_diagnostics = 'system healthy'

        self.startup_mode = False

        self.update_system_on = False
        
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

            

    
           
