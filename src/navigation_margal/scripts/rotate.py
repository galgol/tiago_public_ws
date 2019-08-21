#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from tf.transformations import *
from geometry_msgs.msg import Pose #change to tiago
pi = 3.1415926535897
from geometry_msgs.msg import Quaternion


def localize():
##for each service that we want to use (all the rqt call we do) we can do it  with : 
##rospy.wait_for_service('global_localization')
##the wait_for_service func - will make sure the service is up - otherwise we will get notification (i hope so)
##global_localization = rospy.ServiceProxy('global_localization', Empty)
##this call trying to connect to the node and sending the message to the server - in our case - empty (but we can maybe   use it ##for the arm)'''
 
    rospy.wait_for_service('global_localization')
    global_localization = rospy.ServiceProxy('global_localization', Empty)
    global_localization()
  
def rotate():

    #Starts a new node
    #rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate = rospy.Rate(1)
    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = 20
    angle = 90
    clockwise = True
    #Converting from angles to radians
    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    i = 0

    while (i < 9):
	    # Setting the current time for distance calculus
	    t0 = rospy.Time.now().to_sec()
	    current_angle = 0
	    while(current_angle < relative_angle):
		velocity_publisher.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)

	    print("in iteration num " , i)
            i = i + 1

    print("after while")
    #Forcing our robot to stop
    vel_msg.angular.z = 0
    #while not rospy.is_shutdown():
    velocity_publisher.publish(vel_msg)
    #rate.sleep()
 
    rospy.wait_for_service('move_base/clear_costmaps')
    clear_localization = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
    clear_localization()
    print("clear map")
    

    #rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        localize()
        rotate()
	rospy.spin()
    except rospy.ROSInterruptException:
        pass
