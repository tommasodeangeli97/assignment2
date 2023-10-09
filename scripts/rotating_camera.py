#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState

"""
.. module::rotating_joint
   :platform: Unix
   :synopis: module for rotating the arm_joint where the camera is fixed
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS node that control the rotation of the arm_joint in order to find all the markers in the first room

Publishes to:
    - /roblucci/joint1_position_controller/command(Float64) topic that impose the rotation of the link
    
Subribes to:
    - /roblucci/joint1_position_controller/state(JointControllerState) topic used for retrieving the position of the joint
"""

value_scan = 0
"""
global variable where the position of the arm_joint is stored
"""
def sc_callback(msg):
    """
    callback function for the /roblucci/joint1_position_controller/state
    """
    global value_scan
    value_scan = msg.process_value
            
def inspection():
    """
    main function of the node: it initialises the node and publish on /roblucci/joint1_position_controller/command for rotating the arm always checking the status of the arm
    """
    global value_scan
    rospy.init_node('rotating_joint', anonymous = True)
    joint1_pose_pub = rospy.Publisher('/roblucci/joint1_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/roblucci/joint1_position_controller/state", JointControllerState, sc_callback)
    
    joint1_pose_pub.publish(-3.14)
    
    while value_scan > -3.139:
        print(" value_scan >: ", value_scan)
        
        joint1_pose_pub.publish(-3.14)
        
    joint1_pose_pub.publish(3.0)    
    while value_scan < 2.99:
        print(" value_scan <: ", value_scan)
        
        joint1_pose_pub.publish(3.0)
        
        
    joint1_pose_pub.publish(0.0)
    
if __name__ == '__main__':
    """
    the entry point of the node
    """
    try:
        inspection()
    except rospy.ROSInterruptException:
    	pass
