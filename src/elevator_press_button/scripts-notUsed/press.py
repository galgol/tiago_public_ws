#!/usr/bin/env python
import rospy
#from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal
from std_srvs.srv import Empty
import actionlib
import sys
from copy import copy
from trajectory_msgs.msg import JointTrajectoryPoint #change to tiago

		
def init():
		rospy.init_node('press_button', anonymous=False)
		rospy.loginfo("start arm server")

		#rospy.init_node('trajectory')
		#pub = rospy.Publisher('/arm_controller/follow_joint_trajectory',JointTrajectory,queue_size=10)
	

def points():
		pub = rospy.Publisher('/arm_controller/follow_joint_trajectory',JointTrajectory,queue_size=10)
		trajectory = JointTrajectory()
		point = JointTrajectoryPoint()
		trajectory.header.stamp=rospy.Time.now()
		trajectory.header.frame_id = "/base_link";
		print("here")
		trajectory.joint_names.append("arm_1_joint");
		trajectory.joint_names.append("arm_2_joint");
		trajectory.joint_names.append("arm_3_joint");
		trajectory.joint_names.append("arm_4_joint");
		trajectory.joint_names.append("arm_5_joint");
		trajectory.joint_names.append("arm_6_joint");
		trajectory.joint_names.append("arm_7_joint");
		print("here2")
		point.positions.append(1.0591656143964283)
		point.positions.append(0.48357964958694666)
		point.positions.append(-1.8060198686571673)
		point.positions.append(0.28170960172582893)
		point.positions.append(-1.82175838806662)
		point.positions.append(0.07656840816165061)
		point.positions.append(-2.074393506565951)


		trajectory.points.append(point)
		pub.publish(trajectory)
		print("here3")

if __name__ == '__main__':
        try:
                # Testing our function
	        #before()
                #points()
	        #start()
	        init()
		points()
		#arm.start()
		#arm.wait(15.0)
	        rospy.spin()
        except rospy.ROSInterruptException:
                pass
