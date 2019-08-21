#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import std_msgs
from rospy import Publisher
import rotate
import map_navigation
#import os
#import sys
#import subprocess
#from subprocess import Popen, PIPE


#rospy.init_node('map_navigation_node', anonymous=True)
#def load_nav():
	#print("start node")

	#try:
	#	command_string = "rosrun navigation_margal map_navigation.py"
	#	robot_description = subprocess.check_output(
	#	    command_string, shell=True, stderr=subprocess.STDOUT)
	#except subprocess.CalledProcessError as process_error:
	#	rospy.logfatal('Failed to run navigation node: \n%s', process_error.output)
	#	sys.exit(1)
	#node_pid = os.spawnv(os.P_NOWAIT, "rosrun", ["<navigation_demo>", "<map_navigation_node>", ""])	
	#so.kill(node_pid, signal.SIGINT)

	

def send_goals():
	print("start node")
	map_navigation.map_navigation()
	rotate.localize()
	rotate.rotate()
	i=0

	while (i<4) :
	    print("here" , i)
	    rotate.rotate()
            map_navigation.map_navigation()
	    i = i + 1


	
	#so.kill(node_pid, signal.SIGINT)

if __name__ == '__main__':
    try:
        #rospy.init_node('load_nav', anonymous=True)
        #load_nav()
	send_goals()
    except rospy.ROSInterruptException:
        pass
