#!/usr/bin/env python
import rospy
import actionlib
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import rotate


class map_navigation():

  def __init__(self):


    rospy.init_node('map_navigation', anonymous=False)


    # declare the coordinates of interest

    self.xCafe = 7.08559799194
    self.yCafe = 2.62427425385
    self.xOffice1 = 15.6427402496
    self.yOffice1 = -0.102787494659
    self.xOffice2 = 17.624
    self.yOffice2 = -2.048
    self.xOffice3 = 22.1796607971
    self.yOffice3 =  2.95407390594
    self.xOffice4 = 23.970
    self.yOffice4 = 0.174
    self.xOffice5 = -1.822 #
    self.yOffice5 = -1.614
    self.xOffice6 = 3.145 #
    self.yOffice6 = -2.159
    self.xOffice7 = 3.788 #
    self.yOffice7 = -5.046
    self.xOffice8 = 13.0782737732 ##
    self.yOffice8 = -1.9562921524

    self.goalReached = False

    self.navigate = rospy.Service("/navigate_room", Empty, self.go_to_room)

    #self.goalReached = False
    # initiliaze


  def go_to_room(self, req):
	self.go_to_room3()
	self.navigate_to()

	return {}

  def go_to_room3(self):
	self.goalReached = self.moveToGoal(self.xOffice3, self.yOffice3)



  def navigate_to(self):

    if (self.goalReached):
      rospy.loginfo("Congratulations!")

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
