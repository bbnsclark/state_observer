#!/usr/bin/python

import os
import time
import subprocess

class SystemManager:

    def __init__(self):

        pass

    def start_package(self, package, script):

        cmd = 'screen -d -m -S ' + package + ' /home/nvidia/rover/src/rover_launcher/bin/' + script 

        os.system(cmd)


    def stop_package(self, package):

        cmd = 'screen -ls | awk -vFS=\'\t|[.]\' \'/ ' + package + '/ {system("screen -S "$2" -X quit")}\' '

        #cmd = 'screen -X -S '+ package + ' kill'

        os.system(cmd)


    def restart_package(self, package, script):

        self.stop_package(package)

        time.sleep(1)

        self.start_package(package, script)

        time.sleep(1)


    def check_package(self, pkg):

        if (subprocess.check_output('screen -ls', shell=True)).find(pkg) == -1:
    
            return True

        else:

            return False 


    def start_stack(self, pkg_list):

        if pkg_list != []: 

            for pkg in pkg_list:

                self.start_package(pkg['name'], pkg['script'])


    def restart_stack(self, pkg_list):

        if pkg_list != []: 

            for pkg in pkg_list:

                self.restart_package(pkg['name'], pkg['script'])


    def stop_stack(self, pkg_list):

        if pkg_list != []: 
        
            for pkg in pkg_list:

                self.stop_package(pkg['name'])

                time.sleep(1) 

    def check_stack(self, pkg_list):

        failed = []

        if pkg_list != []: 
        
            for pkg in pkg_list:

                if self.check_package(pkg['name']):

                    failed.append(pkg['name'])

        return failed


 
