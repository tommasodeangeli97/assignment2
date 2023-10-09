#!/usr/bin/env python

import rospy
import time
import math
import random
import rospkg
from std_msgs.msg import Bool, String, Float64
from armor_api.armor_client import ArmorClient
from assignment2.srv import RoomInformation

"""
.. module::map_node
   :platform: Unix
   :synopis: a module to creat the ontology where the robot moves
   
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS node that load an ontology and creates another with different instances, data propriety and object propriety using the armor_api client service; after that it send a boolean value to indicate the right functioning of the procedure; all the procedure is done after that the markers are seen and checked.

Publishes to:
    - /load(bool) indicates that the map is updated and ready to be used
	
"""

client = ArmorClient("example", "ontoRef")
"""
client variable that opens the armor client
"""
mark_list = []
"""
variable for saving the markers that are seen by the marker_server node
"""

def view_time(time_list):
    """
    function that clear the queried time
    
    Args:
        -time_list the list of queried time stamp
    Returns:
        the time_stamp usable value
    """
    timestamp = ''
    for i in time_list:
        for element in range(1, 11):
            timestamp=timestamp+i[element]
    return timestamp    

def aruco_callback(data):
    global mark_list
    
    for word in data.data.split():
        if word.isdigit():
            num = int(word)
            if 10 < num < 18 and num not in mark_list:
                mark_list.append(num)
                print(mark_list)

def visited(room_list):
    """
    function for updating the data proprierty 'visitedAt' of the rooms 
    
    Args:
        -room_list the list of the rooms
    """
    for i in list(set(room_list)):
        if i.startswith('R'):
            client.manipulation.add_dataprop_to_ind("visitedAt", i, "Long", str(math.floor(time.time())))
            rospy.sleep(random.uniform(0.5,1.5))

def create_map():
    """
    function that creates the ontology loading the already existing one and modifing it with the information given by the markers, when it is done it saves and other otology 
    """
    global mark_list
    
    print( " Load the map ", mark_list )
    client.call('LOAD','FILE','',['/root/ros_ws/src/assignment2/topological_map/topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    
    rospy.wait_for_service('/room_info')
    roomInfo_srv = rospy.ServiceProxy('/room_info', RoomInformation)
    individuals = []
    for i in mark_list: 
        info = roomInfo_srv(i)
        roomID = info.room
        individuals.append(roomID)
        room_X = info.x
        room_Y = info.y

        client.manipulation.add_dataprop_to_ind("X_point", roomID , "float", str(room_X))
        client.manipulation.add_dataprop_to_ind("Y_point", roomID , "float", str(room_Y))
        
        for c in info.connections:
            c.through_door
            individuals.append(c.through_door)
            client.call('ADD','OBJECTPROP','IND',['hasDoor', roomID, c.through_door])
    
    #set() method is used to convert any iterable to sequence of distinct elements
    client.call('DISJOINT','IND','', list(set(individuals)))
    client.call('REASON','','',[''])
    
    visited(individuals)
    
    client.call('REASON','','',[''])
    client.call('ADD','OBJECTPROP','IND',['isIn', 'Robot1', 'E'])
    #Update robot now property
    client.call('REASON','','',[''])
    req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    oldtimerobot=view_time(req.queried_objects)
    newtime=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', newtime, oldtimerobot])
    
    client.call('SAVE','','',['/root/ros_ws/src/assignment2/topological_map/new_map.owl'])

def loading():
    """
    initial function that checks if the list of the markers is complete; ones it is complete it calls the create_map function for creating the ontology 
    """
    global mark_list
    #rospy.init_node('map_node', anonymous=True)
    pub = rospy.Publisher("load", Bool, queue_size=10)
    sub = rospy.Subscriber("/marker_publisher/target", String, aruco_callback)
    pub.publish(False)
    
    print (" Loading the map ")
    
    while not rospy.is_shutdown():
        if len(mark_list)>=7:
            print(" all markers up ")
            sub.unregister()
            create_map()
            pub.publish(True)
            rospy.sleep(3)
            print (" Map ready ")
            
            rospy.signal_shutdown('map_node')
        else:
            print(" I have this markers: ", mark_list)

if __name__ == '__main__':
    """
    the entry point of the node
    """
    rospy.init_node('map_node', anonymous=True)
    
    try:
        
        loading()
    except rospy.ROSInterruptException:
    	pass
