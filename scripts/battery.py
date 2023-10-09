#!/usr/bin/env python

import rospy
import roslib
import random
import threading
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String

"""
.. module::battery_node
   :platform: Unix
   :synopis: a module for publishing the status of the battery
   
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS node that change the boolean variable 'battery' wich indicate the battery level of the robot; the node waits a random time after the robot leaves the charging room and publish the True value to indicate the discharging of the battery. The node runs in concorrence with the others using different thread

Publishes to:
	- /batttrigger boolean value
	
Subribes to:
	- /room bulean value that indicates the presence of the robot in the charging room ('E' room)
	
"""

room = False
"""
variable room: boolean variable, True if the robot is actually in the 'E' room and can charge itself
"""

def pos_callback(data):
	"""
	callback function for updating the value of 'room'
	"""
		
	global room
	room = data.data
			
def cancel_goal():
	"""
	function for deleting the goal via move_base action client
	"""
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	client.cancel_all_goals()

def charge():
	"""
	main function in which the value of 'room' is checked and if it is True (the robot is in the right room) the robot charge for the indicated time and publishs that the battery is full -/batttrigger=False, if it is False the robot discharge for a random time and publishes the low battery value -/batttrigger=True
	"""
	global room
	#rospy.init_node('battery_node', anonymous=True)
	pub = rospy.Publisher('batttrigger', Bool, queue_size=10)
	rospy.Subscriber('room', Bool, pos_callback)
	while not rospy.is_shutdown():
		
		if room == True:
			rospy.sleep(3)
			batt = False
			print(" recharged ")
			pub.publish(batt)
		else:
			autonomy = random.uniform(150,180)
			rospy.sleep(autonomy)
			batt = True
			cancel_goal()
			pub.publish(batt)
			
			print(" need recharge ")	
		

if __name__ == '__main__':
	"""
	The entry point of the node, the thread is initialized
	"""
	rospy.init_node('battery_node', anonymous=True)
	try:
		charge()
	except rospy.ROSInterruptException:
		pass	
