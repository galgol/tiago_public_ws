#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import rotate


class map_navigation():

  def choose(self):


    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|PRESSE A KEY:")
    rospy.loginfo("|'0': roomN enterance")
    rospy.loginfo("|'1': roomN 2 door")
    rospy.loginfo("|'2':  after corner")
    rospy.loginfo("|'3': room3")
    rospy.loginfo("|'4': elevator door")
    rospy.loginfo("|'5': roomL")
    rospy.loginfo("|'6':  before roomN 2 door")
    rospy.loginfo("|'7': localize ")
    rospy.loginfo("|'q': Quit ")
    rospy.loginfo("|-------------------------------|")
    rospy.loginfo("|WHERE TO GO?")

    choice_from_srv = 'exit_room'
    reader = {"roomN enterance":0,
  			 "roomN 2 door":1,
  			 "after corner":2,
  			 "room3":3,
              		 "elevator door":4,
			 "roomL":5,
			 "before roomN 2 door":6,
  			 "localize":7,
  			 "quit":'q'}


    choice = input()
    return choice


    '''def server_str(server_name,server_func):
  	rospy.wait_for_service(server_name)
  	were_to_str = rospy.ServiceProxy(server_name, server_func)
  	try:
  		resp = were_to_str()
  	except rospy.ServiceException as exc:
  		print("Service did not process request: " + str(exc))

  	return resp'''		




  def __init__(self):

    # declare the coordinates of interest

    self.xCafe = 7.08559799194
    self.yCafe = 2.62427425385
    self.xOffice1 = 15.6427402496
    self.yOffice1 = -0.102787494659
    self.xOffice2 = 17.624
    self.yOffice2 = -2.048
    self.xOffice3 = 22.1796607971 
    self.yOffice3 =  2.95407390594
    self.xOffice4 = 23.9195709229
    self.yOffice4 = -0.448810100555
    self.xOffice5 = 2.92525863647 
    self.yOffice5 = -3.48975753784
    self.xOffice6 = 13.0782737732   
    self.yOffice6 = -1.9562921524

    self.goalReached = False
    # initiliaze
    rospy.init_node('map_navigation', anonymous=False)
    choice = self.choose()

    if (choice == 0):

      self.goalReached = self.moveToGoal(self.xCafe, self.yCafe)

    elif (choice == 1):

      self.goalReached = self.moveToGoal(self.xOffice1, self.yOffice1)

    elif (choice == 2):

      self.goalReached = self.moveToGoal(self.xOffice2, self.yOffice2)

    elif (choice == 3):

      self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)
    
    elif (choice == 4):

      self.goalReached = self.moveToGoal(self.xOffice4, self.yOffice4)

    elif (choice == 5):

      self.goalReached = self.moveToGoal(self.xOffice5, self.yOffice5)
    
    elif (choice == 6):

      self.goalReached = self.moveToGoal(self.xOffice6, self.yOffice6)

    elif (choice == 7):
	
	rotate.rotate()
	rospy.spin()
        
    if (choice!='q' and choice!= 7):

      if (self.goalReached):
        rospy.loginfo("Congratulations!")
        #rospy.spin()

      else:
        rospy.loginfo("Hard Luck!")

  def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep()

  def moveToGoal(self,xGoal,yGoal):

      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")

      '''while(not ac_gaz.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base_simple action server to come up")'''
	
      goal = MoveBaseGoal()
     

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/

      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)	
      ac.wait_for_result(rospy.Duration(60))

      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("You have reached the destination")
              return True
		
      else:
              rospy.loginfo("The robot failed to reach the destination")
              return False

if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
