# tiago_public_ws
tiago navigation project
*run project steps:

-first treminal: roslaunch navigation_margal tiago_navigation_public_margal.launch

-second terminal: rosrun navigation_margal navWithLocal.py

-third terminal: rosservice call /navigate_room

*run arm trajectory project:

-first terminal: roslaunch elevator_press_button tiago_navigation_public_margal.launch

-second terminal: rosrun elevator_press_button elevator_press_button_node
