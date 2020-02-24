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
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from state_observer.srv import SetMode
from state_observer.msg import Diagnostics
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

import dynamic_reconfigure.client

class Observer:

    def adjust_keys_for_platform_suffix(self, dictionary):

        for k,v in dictionary.items():
            new_key = k + self.platform_suffix
            print('Old key: ' + k + ', key with suffix: ' + new_key)
            del dictionary[k]
            dictionary[new_key] = v


    def adjust_strings_for_platform_suffix(self, string_array):

        result = []
        for name in string_array:
            result.append(name + self.platform_suffix)
        return result


    def __init__(self, is_sitl, is_airsim):

        self.manager = SystemManager(is_sitl)

        if is_sitl:

            nodes_filename = 'nodes_sitl.yaml'

        else:

            nodes_filename = 'nodes.yaml'
        
        with open(sys.path[0] + '/../config/' + nodes_filename, 'r') as stream:

            NODES = yaml.load(stream)

        # Add any key suffix (for multi-platform configurations)
        self.platform_suffix = rospy.get_param('~platform_suffix', "")
        print('platform_suffix = ' + self.platform_suffix)
        self.adjust_keys_for_platform_suffix(NODES)

        for k,v in NODES.items():
                    
            # Let all nodes know their names - which are their keys in the NODES map
            v['name'] = k

            v['topic_type'] = eval(v['topic_type'])

        # Setting dynamic parameters for the dwa planner
        self.global_dwa_params = {
            'acc_lim_x': 0.5,
            'max_vel_x': 0.5, 
            'min_vel_x': -0.15, 
            'max_vel_trans': 0.5, 
            'min_vel_trans': -0.15,  

            'max_vel_theta': 0.5, 
            'min_vel_theta': -0.5,
            'acc_lim_theta': 1.5,

            'sim_time': 3.5,
            'vx_samples': 10,
            'vth_samples': 10,

            'xy_goal_tolerance': 1.0, 
            'yaw_goal_tolerance': 0.2,

            'path_distance_bias': 32.0,
            'goal_distance_bias': 20.0,
            
            }

        self.transition_dwa_params = {
            'acc_lim_x': 0.25, 
            'max_vel_x': 0.35, 
            'min_vel_x': -0.1, 
            'max_vel_trans': 0.35, 
            'min_vel_trans': -0.1,  

            'max_vel_theta': 0.5, 
            'min_vel_theta': -0.5,
            'acc_lim_theta': 1.0,

            'sim_time': 3.5,
            'vx_samples': 10,
            'vth_samples': 10,

            'xy_goal_tolerance': 0.5, 
            'yaw_goal_tolerance': 0.15
            
            }
        
        self.inertial_dwa_params = {
            'acc_lim_x': 0.25, 
            'max_vel_x': 0.3,
            'min_vel_x': -0.05, 
            'max_vel_trans': 0.3,
            'min_vel_trans': -0.05,  

            'max_vel_theta': 0.35, 
            'min_vel_theta': -0.35,
            'acc_lim_theta': 0.75,

            'sim_time': 3.5,
            'vx_samples': 10,
            'vth_samples': 10,

            'xy_goal_tolerance': 0.35, 
            'yaw_goal_tolerance': 0.15
            
            }

        if is_sitl == False:

            print("Hardware mode")

            self.common_nodes = {k:v for k,v in NODES.items() if k in ['roscore', 'video', 'state_obs','april_tags', 'rosbridge', 'realsense', 'imu', 'drive', 'lidar', 'ekf_inertial', 'navigation']}.values()

            self.global_nodes = {k:v for k,v in NODES.items() if k in ['map_tf', 'gps_driver', 'gps_conv', 'control_global']}.values()

            self.transition_nodes = {k:v for k,v in NODES.items() if k in ['map_tf']}.values()

            self.inertial_nodes = {k:v for k,v in NODES.items() if k in ['map_inertial', 'map_local']}.values()

        else:

            if is_airsim:

                print("AirSim mode")

                common_node_names = ['roscore', 'video', 'state_obs', 'april_tags', 'rosbridge', 'sitl', 'ekf_inertial', 'navigation', 'rviz']
                common_node_names = self.adjust_strings_for_platform_suffix(common_node_names)

                global_node_names = ['map_tf', 'gps_driver_airsim', 'nav_sat', 'control_global']
                global_node_names = self.adjust_strings_for_platform_suffix(global_node_names)

                print('Common node names: ')
                print(common_node_names)

                self.common_nodes = {k:v for k,v in NODES.items() if k in common_node_names}.values()
                self.global_nodes = {k:v for k,v in NODES.items() if k in global_node_names}.values()

                transition_node_names = ['map_tf']
                transition_node_names = self.adjust_strings_for_platform_suffix(transition_node_names)
                self.transition_nodes = {k:v for k,v in NODES.items() if k in transition_node_names}.values()

                inertial_node_names = ['map_inertial', 'map_local', 'rviz_inertial']
                inertial_node_names = self.adjust_strings_for_platform_suffix(inertial_node_names)
                self.inertial_nodes = {k:v for k,v in NODES.items() if k in inertial_node_names}.values()

                self.global_nodes = {k:v for k,v in NODES.items() if k in ['gps_driver_airsim', 'gps_conv', 'control_global']}.values()

                self.inertial_nodes = {k:v for k,v in NODES.items() if k in ['map_inertial', 'map_local']}.values()
            
            else:

                print("Gazebo mode")

                self.common_nodes = {k:v for k,v in NODES.items() if k in ['roscore', 'video', 'state_obs', 'april_tags', 'rosbridge', 'sitl', 'ekf_inertial', 'navigation', 'rviz']}.values()

                self.global_nodes = {k:v for k,v in NODES.items() if k in ['map_tf', 'gps_driver', 'gps_conv', 'control_global']}.values()

                self.inertial_nodes = {k:v for k,v in NODES.items() if k in ['map_inertial', 'map_local']}.values()

                self.transition_nodes = {k:v for k,v in NODES.items() if k in ['map_tf']}.values()

        self.system_states = ['idle', 'broadcasting', 'fault']

        self.system_modes = ['', 'inertial', 'global', 'transition']

        self.system_nodes = {'': [], 'inertial': self.inertial_nodes, 'global': self.global_nodes, 'transition': self.transition_nodes}

        self.system_dwa_params = {'': [], 'inertial': self.inertial_dwa_params, 'global': self.global_dwa_params, 'transition': self.transition_dwa_params}

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

        self.reconf_dwa = dynamic_reconfigure.client.Client('/MOVE/DWAPlannerROS')

        nodes  = []

        all_nodes = (self.common_nodes + self.system_nodes[new_mode])

        print(all_nodes)

        for node in all_nodes:
            
            nodes.append(node['name'])

        cur_nodes = self.manager.get_active_packages()

        to_be_stopped = [x for x in cur_nodes if x not in nodes]

        for stack in to_be_stopped:

            self.manager.stop_package(stack)

        self.current_system_mode = new_mode

        to_be_started_keys = [x for x in nodes if x not in cur_nodes]

        to_be_started = [k for k in all_nodes if k['name'] in to_be_started_keys]

        self.manager.start_stack(to_be_started)

        self.reconf_dwa.update_configuration(self.system_dwa_params[new_mode])

        return 'mode set to: ' + str(self.current_system_mode)


    def system_reset(self):

        self.count = 0
        
        self.startup_mode = True

        self.manager.restart_stack(self.common_nodes + self.system_nodes[self.current_system_mode])

            

    
           
