#!/usr/bin/python

import time
import numpy as np
from datetime import datetime
from math import pi, cos, sin
from observer import Observer

import tf
import rospy
import rosnode
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix
from state_observer.srv import SetMode
from sensor_msgs.msg import BatteryState
from state_observer.msg import Diagnostics
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Quaternion, Twist

class Node:

    def __init__(self):

        rospy.init_node('STATE')
            
        self.is_sitl = rospy.get_param('~is_sitl', False)
            
        self.is_airsim = rospy.get_param('~is_airsim', False)

        self.observer = Observer(self.is_sitl, self.is_airsim)

        self.rate = 1.0

        self.pub_diag = rospy.Publisher('/system_diagnostics', Diagnostics, queue_size = 1)

        self.srv_cmd_state = rospy.Service('set_mode', SetMode, self.set_mode_callback)

        rospy.wait_for_service('/MOVE/clear_costmaps')
        
        self.clear_costmaps_srv = rospy.ServiceProxy('/MOVE/clear_costmaps', Empty)

        rospy.loginfo('Starting state observer...')

        self.diag = Diagnostics()


    def run(self):

        rospy.loginfo('Starting state broadcast')

        r_time = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            state, status = self.observer.get_system_info()

            self.diag.state = state

            self.diag.status = status

            self.pub_diag.publish(self.diag)

            r_time.sleep()


    def set_mode_callback(self, msg):

        reply = 'set_mode did not receive response from system...'

        if msg.cmd == 'set':

            reply = self.observer.set_system_mode(msg.target_mode)

            self.clear_costmaps_srv(EmptyRequest())

        elif msg.cmd == 'reset':

            self.observer.system_reset()

            reply = "system reset"

        return reply


if __name__ == '__main__':

    try:

        node = Node()

        node.run()

    except rospy.ROSInterruptException:

        pass

    rospy.loginfo('Exiting')
