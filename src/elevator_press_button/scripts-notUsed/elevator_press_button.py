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


class press_button():

	#def before(self):
		##for each service that we want to use (all the rqt call we do) we can do it  with : 
		##rospy.wait_for_service('global_localization')
		##the wait_for_service func - will make sure the service is up - otherwise we will get notification (i hope so)
		##global_localization = rospy.ServiceProxy('global_localization', Empty)
		##this call trying to connect to the node and sending the message to the server - in our case - empty (but we 			can maybe   use it ##for the arm)'''
		#rospy.init_node('map_navigation', anonymous=False)
		#/arm_controller/follow_joint_trajectory/goal

		    
			
	def __init__(self):
		rospy.init_node('press_button', anonymous=False)
		rospy.loginfo("start arm server")
		#rospy.wait_for_service('/arm_controller/follow_joint_trajectory/goal')
		#arm_control = rospy.ServiceProxy('/arm_controller/follow_joint_trajectory/goal', Empty) 
		#arm_control()

		#ac = actionlib.SimpleActionClient('arm_controller', FollowJointTrajectoryGoal)
      		#wait for the action server to come up
     		#while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              		#rospy.loginfo("Waiting for the move_base action server to come up")


		self._client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',
		FollowJointTrajectoryAction,
		)
		self._goal = FollowJointTrajectoryGoal()
        	self._goal_time_tolerance = rospy.Time(0.1)
        	self._goal.goal_time_tolerance = self._goal_time_tolerance
        	server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
		print("here")
        	if not server_up:
			print("not")
            		rospy.logerr("Timed out waiting for Joint Trajectory")
            		rospy.signal_shutdown("Timed out waiting for Action Server")
            		sys.exit(1)

		#self.clear()	
  
	def points(self):
		rospy.loginfo("sending goal")
       		
		jt = JointTrajectory()
		jt.header.stamp=rospy.Time.now()
		jt.header.frame_id = "/base_link";
		#jt.joint_names = ['arm_1_joint',
                  #             	  'arm_2_joint',
                  #             	  'arm_3_joint',
                   #               'arm_4_joint',
                   #               'arm_5_joint',
                  #                'arm_6_joint',
		#	          'arm_7_joint']

		
		#self._goal.trajectory.append(jt)
		print("after append names")

		point = JointTrajectoryPoint()
		point.positions = [1.0591656143964283, 
		     		   0.48357964958694666, 
		     		   -1.8060198686571673,
	             		   0.28170960172582893, 
		     		   -1.82175838806662,
		     		   0.07656840816165061, 
		     		   -2.074393506565951]

		print("after points")
		point.time_from_start = rospy.Duration(4.0)
		self._goal.trajectory.points.append(point)
		
		#jt.points.append(point)
		print("after append points")
		
		

        def start(self):
                self._goal.trajectory.header.stamp = rospy.Time.now()
	        self._client.send_goal(self._goal)
		print("start is here")
	
        def stop(self):
                self._client.cancel_goal()

      	def wait(self, timeout=15.0):
                self._client.wait_for_result(timeout=rospy.Duration(timeout))

      	#def result(self):
              	#return self._client.get_result()

      
        #def clear(self):
                #self._goal = FollowJointTrajectoryGoal()
                #self._goal.goal_time_tolerance = self._goal_time_tolerance


if __name__ == '__main__':
        try:
                # Testing our function
	        #before()
                #points()
	        #start()
	        arm=press_button()
		arm.points()
		arm.start()
		arm.wait(15.0)
	        rospy.spin()
        except rospy.ROSInterruptException:
                pass

