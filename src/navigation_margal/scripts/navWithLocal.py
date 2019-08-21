#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import std_msgs
from rospy import Publisher
import rotate
import map_navigation


def send_goals():
	print("starting node")
		   	
	map_navigation.map_navigation()
	
if __name__ == '__main__':
    try:
	send_goals()
	rospy.spin()
    except rospy.ROSInterruptException:
        pass
