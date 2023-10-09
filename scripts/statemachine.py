#!/usr/bin/env python

import rospy
import roslib
import rospkg
import smach
import smach_ros
import time
import math
import random
import re
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Point
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import Twist
from armor_api.armor_client import ArmorClient

"""
.. module::state_machine
   :platform: Unix
   :synopis: the state machine to controll the behavior of the robot
   
.. moduleauthor:: tommaso de angeli <tommaso.deangeli.97@gmail.com>

ROS state machine implemented with smach library with the purpose of controlling the behaviour of the robot; the robot analyse the environment around him and waits till the map is ready; then the robot must stays mostly in corridors and visites the rooms as they became 'urgent'; when the battery level is low the robot goes back to the initial room, the 'E' room, and charge itself

Publishes to:
	- /room(bool) it indicates that the robot is in the charging room: True value -> the robot is in the room; False value -> the robot is not in the right room
	- /target(Point) the topic where the coordinates of the next room are published
	- /startmove it indicates if the robot is ready to leave the current room
	- /roblucci/joint1_position_controller/command the topic for controlling the movement of the arm_joint
	
Subribes to:
	- /load(bool) it indicates if the map is ready: True value -> the ontology is ready; False value -> the ontology is not ready yet
	- /batttrigger(bool) it indicates if the battery is full or low: True value -> the battery is low; False value -> the battery is full
	- /roblucci/joint1_position_controller/state that indicates the state of the arm_joint
	- /reached that indicates if the robot has reached the requested room
	
"""


ready = False
"""
global variable for the value of the /load topic
"""
battery = False
"""
global variable for the value of the /batttrigger topic
"""
room_d = []
"""
global dictionary where the name and the coordinates of all the rooms are stored
"""
place = False
"""
global variable for storing the /reached topic value
"""
value_scan = 0
"""
global variable for storing the /roblucci/joint1_position_controller/state topic value
"""
wim = []

def callback(data):
	"""
	callback function that changes the 'ready' variable
	"""
	global ready
	ready = data.data
	
def bat_callback(data):
	"""
	callback function that changes the 'battery' variable
	"""
	global battery
	battery = data.data
	
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
	
def visited(nr):
	"""
	function that update the data propriety 'visitedAt'

	Args:
		-r_n the new room where the robot is
	"""
	client = ArmorClient("example", "ontoRef")
	nrc = []
	if nr[0] == "R1":
		print(" nrc constructed 1")
		nrc.append('R1')
	if nr[0] == "R2":
		print(" nrc constructed 2")
		nrc.append('R2')
	if nr[0] == "R3":
		print(" nrc constructed 3")
		nrc.append('R3')
	if nr[0] == "R4":
		print(" nrc constructed 4")
		nrc.append('R4')
	if nr[0] == "C1":
		print(" nrc constructed 5")
		nrc.append('C1')
	if nr[0] == "C2":
		print(" nrc constructed 6")
		nrc.append('C2')
	if nr[0] == "E":
		print(" nrc constructed 7")
		nrc.append('E')
	print(" nrc= ", nrc)	
		
	
	room_time = client.call('QUERY','DATAPROP','IND',['visitedAt', nrc[0]])
	old_room = view_time(room_time.queried_objects)
	print(" step 3 ")
	now_t = str(math.floor(time.time()))
	client.call('REPLACE','DATAPROP','IND',['visitedAt',nrc[0], 'Long', now_t, old_room])
	client.call('REASON','','',[''])
	rospy.sleep(1)
	print(" visited ")
	return

def check_urgents():
	"""
	function that queries if there are 'urgent' room and checkes if the list is empty
	
	Returns:
		the list of all the 'urgent' rooms in that moment
	"""
	client = ArmorClient("example", "ontoRef")
	client.call('REASON','','',[''])
	calling = client.call('QUERY','IND','CLASS',['URGENT'])
	urg_list = []
	for i in calling.queried_objects:
		if "R1" in i:
			print(" urgent R1 ")
			urg_list.append('R1')
		elif "R2" in i:
			print(" urgent R2 ")
			urg_list.append('R2')
		elif "R3" in i:
			print(" urgent R3 ")
			urg_list.append('R3')
		elif "R4" in i:
			print(" urgent R4 ")
			urg_list.append('R4')
		
	if urg_list == []:
		print(" nothing urgent for the moment ", urg_list)
	else:
		print(" we have some urgent rooms ", urg_list)
	return urg_list

def check_pose():
	"""
	function that query the actual position of the robot
	
	Returns:
		the position of the robot
	"""
	client = ArmorClient("example", "ontoRef")
	client.call('REASON','','',[''])
	calling = client.call('QUERY','OBJECTPROP','IND',['isIn', 'Robot1'])
	wim_list = []
	for i in calling.queried_objects:
		if "R1" in i:
			wim_list.append('R1')
		elif "R2" in i:
			wim_list.append('R2')
		elif "R3" in i:
			wim_list.append('R3')
		elif "R4" in i:
			wim_list.append('R4')
		elif "C1" in i:
			wim_list.append('C1')
		elif "C2" in i:
			wim_list.append('C2')
		
		elif "E" in i:
			wim_list.append('E')
	print(" I'm in: ", wim_list)
	rospy.sleep(4)
	return wim_list

def check_near(pos):
	"""
	funtion that queries all the rooms that are near to the one where the robot is
	
	Args:
		-pos: the actual position of the robot
		
	Returns:
		the list of the rooms that share the same door with pos
	"""
	client = ArmorClient("example", "ontoRef")
	client.call('REASON','','',[''])
	calling = client.call('QUERY','OBJECTPROP','IND',['connectedTo', pos[0]])
	near_list = []
	for i in calling.queried_objects:
		if "R1" in i:
			near_list.append('R1')
		elif "R2" in i:
			near_list.append('R2')
		elif "R3" in i:
			near_list.append('R3')
		elif "R4" in i:
			near_list.append('R4')
		elif "C1" in i:
			near_list.append('C1')
		elif "C2" in i:
			near_list.append('C2')
		
		elif "E" in i:
			near_list.append('E')
	print(" near list: ", near_list)
	return near_list

def reached_callback(msg):
	"""
	callback for the /reached topic
	"""
	global place
	place = msg.data
	
def create_coord():
	"""
	function for query the coordinates of the rooms and stores it in the room_d dictionary
	"""
	global room_d
	client = ArmorClient("example", "ontoRef")
	rooms = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
	for i in rooms:
		req=client.call('QUERY','DATAPROP','IND',['X_point', i])
		X=float(take_v(req.queried_objects))
		req=client.call('QUERY','DATAPROP','IND',['Y_point', i])
		Y=float(take_v(req.queried_objects))
		room_d.append({"name":i, "x":X, "y":Y})
	print("created all coordiantes ")

def move(cr, nr):
	"""
	function that changes the position of the robot and change the data proprietry 'now'; it publishes on the /target topic the coordinates and on the /startmove topic that the robot is ready to leave the room
	
	Args:
		-cr: the room that the robot leaves
		-nr: the new room where the robot goes to
	"""
	global room_d, place, wim
	client = ArmorClient("example", "ontoRef")
	pub = rospy.Publisher('target',Point, queue_size= 10)
	st = rospy.Publisher('startmove',Bool, queue_size= 10)
	rospy.Subscriber('reached',Bool, reached_callback)
	print(" i'm moving from ", cr)
	print(" i'm moving to ", nr)
	for i in room_d:
		if i["name"] == nr[0]:
			print(" allright nr: ", i["name"])
			coord = Point()
			coord.x = i["x"]
			coord.y = i["y"]
			pub.publish(coord)
			st.publish(True)
			rospy.sleep(3)
		else:
			print(" ", i["name"])
			
	if nr[0] == "R1":
		nrc = 'R1'
	if nr[0] == "R2":
		nrc = 'R2'
	if nr[0] == "R3":
		nrc = 'R3'
	if nr[0] == "R4":
		nrc = 'R4'
	if nr[0] == "C1":
		nrc = 'C1'
	if nr[0] == "C2":
		nrc = 'C2'
	if nr[0] == "E":
		nrc = 'E'
	if place == True:
		print(" arrived ")
		st.publish(False)
		#place = False
		client.call('REPLACE', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', nrc, cr[0]])
		client.call('REASON','','',[''])
		req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
		client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', str(math.floor(time.time())), view_time(req.queried_objects)])
		client.call('REASON','','',[''])
		print(" changed isIn ")
		wim[0] = nrc
		
	else:
		print(" position not reached ")
		move(cr,nr)
		
	place = False
	return

def sc_callback(msg):
	"""
	callback of the /roblucci/joint1_position_controller/state topic
	"""
	global value_scan
	value_scan = msg.process_value
	
def room_scan():
	""""
	This function controls the commands passed to the joints. Joint 1 will rotate 360 degrees to scan the room."
    
	"""
	global value_scan
	
	# Create a publisher for the cmd_vel topic
	print("...")
	joint1_pose_pub = rospy.Publisher('/roblucci/joint1_position_controller/command', Float64, queue_size=10)
	rospy.Subscriber("/roblucci/joint1_position_controller/state", JointControllerState, sc_callback)
	joint1_pose_pub.publish(-3.14)    
	while value_scan > -3.139:
		#print(" value_scan >: ", value_scan)        
		joint1_pose_pub.publish(-3.14)        
	joint1_pose_pub.publish(3.0)    
	while value_scan < 2.99:
		#print(" value_scan <: ", value_scan)     
		joint1_pose_pub.publish(3.0)      
	joint1_pose_pub.publish(0.0)
	return

def take_v(s_list):
	s = s_list[0]
	match = re.search(r"(\d+\.\d+)", s)

	if match:
		value = float(match.group(1))
		return(value)

class WaitMap(smach.State):
	"""
	Class that defines the 'RECIEVING_MAP' state, the robot just waits untill the variable 'ready' becames 'True' in that case the map is loaded and the robot leaves the 'E' room to go to the 'C1' corridor; it changes the state
	
	Args:
		-smach.State: base state interface
		
	Returns:
		-map_not_redy: transition condition that maintain the 'RECIEVING_MAP' status active
		-map_redy: transistion condition that change status from 'RECIEVING MAP' to 'PATROLLING_CORRIDOR'
	"""
	#initial state, the waiting for the map
	def __init__(self):
		smach.State.__init__(self, outcomes=['map_not_ready','map_ready'])
		
	def execute(self, userdata):
		global ready, room_d, wim
		client = ArmorClient("example", "ontoRef")
		
		rospy.Subscriber("load", Bool, callback)
		
		if ready == False:
			print("I'm still waitnig")
			rospy.sleep(2)
			return 'map_not_ready'
			
		else:
			print(" uploading the map ") 
			client.call('LOAD','FILE','',['/root/ros_ws/src/assignment2/topological_map/new_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
			create_coord()
			wim = check_pose()
			nr = ["E"]
			move(wim, nr)
			#visited(nr)
			#room_scan()
			return 'map_ready'			 
			

class PatrollingCorridor(smach.State):
	"""
	Class that defines the 'PATROLLING_CORRIDOR' state: it checks if the battery is low, in that case the status changes to 'BATTERY_LOW', if not, the robot checks its position, if there are urgent rooms and which are the room accessible; if there are urgent rooms and they are accessible the robot changes its position and the status changes to 'PATROLLING_ROOM'; if there are urgent rooms but the robot isn't near to them it checks which is the room that is near to the urgent rooms and change its position to this room, it remains in the 'PATROLLING_CORRIDOR' status; if there are no urgent rooms the robot prefers to stay in corridors, it changes corridors and remains in the 'PATROLLING_CORRIDOR' status
	
	Args:
		-smach.State: base state interface
		
	Returns:
		-still_corridor: transition condition that maintain the 'PATROLLING_CORRIDOR' status
		-in_room: transition condition that changes to 'PATROLLING_ROOM' status
		-allert_battery: transition condition that changes to 'BATTERY_LOW' status
	"""
	#state where the robot moves in the corridors and check if there are urgent room and in case goes to them
	def __init__(self):
		smach.State.__init__(self, outcomes=['still_corridor','in_room', 'allert_battery'])
		
	def execute(self, userdata):
		global battery
		
		if battery == True:
				print(" the battert is low, i need charge ")
				return 'allert_battery'
		else:
			
			n_c = ''
			wim = check_pose()
			print(" wim = ",wim)
			wicg = check_near(wim)
			atu = check_urgents()
			c1 = 'C1'
			c2 = 'C2'
			e = 'E'
			
			if atu == []:
				print(" let's stay in corridords so ")
				if wim[0] == c1:
					
					n_c = ["C2"]
					print(" next room is ", n_c[0])
					move(wim,n_c)
					#visited(n_c)
					print(" next corridor ")
					return 'still_corridor'
				elif wim[0] == c2:
					
					n_c = ["C1"]
					print(" next room is ", n_c)
					move(wim,n_c)
					#visited(n_c)					
					print(" next corridor ")
					return 'still_corridor'
					
				elif wim[0] == e:
					corridors = ["C2","C1"]
					n_c = random.choices(corridors, weights = [1,1])
					print(" next room is ", n_c)
					move(wim,n_c)
					#visited(n_c)					
					print(" next corridor ")
					return 'still_corridor'
				
			else:
				n_r = []
				room_set = set(wicg) & set(atu)
				for i in room_set:
					if "R1" in i:
						n_r.append("R1")
					elif "R2" in i:
						n_r.append("R2")
					elif "R3" in i:
						n_r.append("R3")
					elif "R4" in i:
						n_r.append("R4")
					elif "C1" in i:
						n_r.append("C1")
					elif "C2" in i:
						n_r.append("C2")
					 
				if n_r:
					print(" the nearest urgent room is ", n_r)
					move(wim,n_r)
					visited(n_r)
					
					return 'in_room'
				else:
					room_set2 = set(check_near(atu)) & set(wicg)
					for i in room_set2:
						if "R1" in i:
							n_r.append("R1")
						elif "R2" in i:
							n_r.append("R2")
						elif "R3" in i:
							n_r.append("R3")
						elif "R4" in i:
							n_r.append("R4")
						elif "C1" in i:
							n_r.append("C1")
						elif "C2" in i:
							n_r.append("C2")				
					
					move(wim,n_r)											
					
					return 'still_corridor'
						
		
class PatrollingRoom(smach.State):
	"""
	Class that define the 'PATROLLING_ROOM' state: the robot checks if the battery is low, in that case the status changes to 'BATTERY_LOW', if not, it scans the room, then checks in which room it is and which is the near room, it moves to that room and change the status to 'PATROLLING_CORRIDOR'
	
	Args:
		-smach.State: base state interface
		
	Returns:
		-back_to_corridor: transition condition that changes the status in 'PATROLLING_CORRIDOR'
		-allert_battery: transition condition that changes to 'BATTERY_LOW' status
	"""
	#state where the robot stays in the room for a bit 
	def __init__(self):
		smach.State.__init__(self, outcomes=['back_to_corridor', 'allert_battery'])
		
	def execute(self, userdata):
		global battery
		
		if battery == True:
			print(" the battert is low, i need immediate charge ")
			return 'allert_battery'
		else:
			wim = check_pose()
			wicg = check_near(wim)
			room_scan()
			print(" I have finished here ")
			move(wim,wicg)
			
			return 'back_to_corridor'
			

class BatteryLow(smach.State):
	"""
	Class that defines the 'BATTERY_LOW' status, it checks if the 'battery' variable is still 'True', in that case it checks the actual position and moves consequently remaining in the 'BATTERY_LOW' status: if the robot is in 'C1' it moves to the 'E' room and publish on the /room topic; if the robot is not in 'C1' it chooses the most rapid path to go to the 'E' room and changes one room at time; when the 'battery' becames 'False' the robot goes to the 'C1' room and change to 'PATROLLING_CORRIDOR' state
	
	Args:
		-smach.State: base state interface
	
	Returns:
		-back_to_work: transition condition that changes to 'PATROLLING_CORRIDOR'
		-allert_battery: transition condition that maintain the 'BATTERY_LOW' status
	"""
	#state triggered whenever the battery is low
	def __init__(self):
		smach.State.__init__(self, outcomes=['back_to_work', 'allert_battery'])
	
	def execute(self, userdata):
		global battery
		wim = check_pose()
		pub=rospy.Publisher('room', Bool, queue_size=10)
		if battery == True:
			
			if wim[0] == 'C1' or wim[0] == 'C2':
				home = ["E"]
				move(wim,home)
				#visited(home)
				#pub=rospy.Publisher('room', Bool, queue_size=10)
				pub.publish(True)
			elif wim[0] == 'E':
				print(" charging ")
				pub.publish(True)
				rospy.sleep(10)
			else:
				corridors = ['C2', 'C1']
				wicg = check_near(wim)
				n_r = []
				room_set = set(wicg)&set(corridors)
				for i in room_set:
					if "R1" in i:
						n_r.append("R1")
					elif "R2" in i:
						n_r.append("R2")
					elif "R3" in i:
						n_r.append("R3")
					elif "R4" in i:
						n_r.append("R4")
					elif "C1" in i:
						n_r.append("C1")
					elif "C2" in i:
						n_r.append("C2")
					
				print(" going to charge ")
				move(wim,n_r)
				#visited(n_r)
				
			return 'allert_battery'
		else:
			print(" Let's start again ")
			
			corridors = ["C2","C1"]
			n_c = random.choices(corridors, weights = [1,1])
			move(wim,n_c)
			#visited(next)
			
			pub.publish(False)
			return 'back_to_work'
			
			
def main():
	"""
	the main function that initialises the node and the state machine, it declares all the conjunctions between the state, it declares the subscribers
	"""
	#the main function
	rospy.sleep(5)
	rospy.init_node('state_machine')
	 
	sm = smach.StateMachine(outcomes=['Interface'])
	
	with sm:
		# Add states to the container
		smach.StateMachine.add('RECIEVING_MAP', WaitMap(), 
			transitions={'map_not_ready':'RECIEVING_MAP', 'map_ready':'PATROLLING_CORRIDOR'})
		smach.StateMachine.add('PATROLLING_CORRIDOR', PatrollingCorridor(), 
			transitions={'still_corridor':'PATROLLING_CORRIDOR', 'in_room':'PATROLLING_ROOM', 'allert_battery':'BATTERY_LOW'})
		smach.StateMachine.add('PATROLLING_ROOM', PatrollingRoom(), 
			transitions={'back_to_corridor':'PATROLLING_CORRIDOR', 'allert_battery':'BATTERY_LOW'})
		smach.StateMachine.add('BATTERY_LOW', BatteryLow(), 
			transitions={'back_to_work':'PATROLLING_CORRIDOR', 'allert_battery':'BATTERY_LOW'})
			
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	#rospy.Subscriber("load", Bool, callback)
	rospy.Subscriber("batttrigger", Bool, bat_callback)
	outcome = sm.execute()
	rospy.spin()
	sis.stop()
	
if __name__ == '__main__':
	"""
	the entry point for the module
	"""
	#rospy.init_node('state_machine')
	main()
		
	
