#!/usr/bin/env python

import rospy
import actionlib

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

"""
.. module::move_base_cliet_py
   :platform: Unix
   :synopis: module for moving the robot giving the goal to MoveBase
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS node that implements a simple move base action client, it revieves the goal from the topic and calls the client, it works in a different thread respect to the state machine

Publishes to:
	- /reached(Bool) it indicates when the robot arrives to the goal
	
Subribes to:
	- /target(Point) the coordinates of the goal
	- /startmove(Bool) it indicates when the robot is ready to move
"""

coord = Point()

"""
global variable where the y coordinate is stored
"""
pub = rospy.Publisher('reached', Bool, queue_size = 10)
"""
global publisher on the topic /reached
"""
st_move = False
"""
global variable where the value of /startmove is stored
"""


def boolcallback(arg):
    """
    callback for the /startmove topic
    """
    global st_move
    st_move = arg.data

def callback(msg):
    """
    callback for the /target topic
    """
    global coord
    coord.x = msg.x
    coord.y = msg.y
    print(" coord: x & y ", coord.x, coord.y)

def movebase_client():
    """
    function that implement the simple action client, it sends the goal and waits for the response, once the robot is arrived to the indicated position it publishes on the /reached topic
    """
    global coord, st_move
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    new_coord = rospy.Subscriber('target', Point, callback)
    start_move = rospy.Subscriber('startmove', Bool, boolcallback)
    if st_move == True:
        print(" ready to move ")
        # Declare the client and connect to move_base server
        
        client.wait_for_server()

        # Create a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map' # Note: the frame_id must be map
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = coord.x
        goal.target_pose.pose.position.y = coord.y
        goal.target_pose.pose.orientation.w = 1.0

        # Send the goal
        client.send_goal(goal)

        # Wait for the server to finish performing the action (you can set a timeout here)
        wait = client.wait_for_result(timeout=rospy.Duration(100.0))

        if not wait:
            rospy.logerr("Failed, position not reached")
            
            pub.publish(False)
        
        else:
            print(" arrived ")
            pub.publish(True)
            st_move = False
            return client.get_result()
    
    else:
        print(" not ready to move jet ")
        rospy.sleep(2)
  

if __name__ == '__main__':
    """
    entry point of the code, it initialises the thread
    """
    rospy.init_node('movebase_client_py')
    while not rospy.is_shutdown():
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
        rospy.sleep(2)
    
    

