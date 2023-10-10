# Assignment2 - FSM patrolling robot

# *Introduction*
This repository contains the the ROS package for the upgrade of the first assignment (https://github.com/tommasodeangeli97/finite-state-machine---security-robot).
The robot here is modeled for gazebo and rviz, it moves using move_base and it acquires video stimulus like the markers that are put inside the environment.
To scan the ARUCO markers inside the environment is used the `OpenCV` library, to move the robot in an autonomous way is used `move_base` and to map the the varius rooms is used `gmapping`, all the oters used tools are specified in the repository on top.
The entire documentation of the varius modules that creates the architecture is avaible here 

# *Environment*
![environment](https://github.com/tommasodeangeli97/assignment2/assets/92479113/87131fe7-be04-4662-ad79-99b7beb9a54e)

The invironment is composed by trhee corridors and fours rooms, differently from the previous one the E room (the charging one) is a corridor, all the rules remain the same but due to the fisics of the application the `urgency treshold is ampliated` to 180 seconds.

# *Modules*
*State_machine* 
--
![graph](https://github.com/tommasodeangeli97/assignment2/assets/92479113/962a74d6-5ec0-4a1f-b277-848206fb6c6d)

It's the main behaviour of the robot:

*`RECIEVING_MAP` the initial status where the robot waits for the map, it creates a dictionary where the different rooms are collected with their coordinates.

*`PATROLLING_CORRIDOR` the status where the robot search for `URGENT rooms`; it chooses firstly the nearest ones but if there are not URGENT rooms it preferes to stay in corridors.

*`PATROLLING_ROOM` the robot is inside a room and it inspects the room before leaving it.

*`BATTERY_LOW` when the battery signal is triggered the robot goes into this status, wherever it is it goes back to the E room to recharge itself; if the robot is inside a room firstly it returns to the nearest corridor and then to the E room.

*Battery module*
--
It is the node that gives the status of the battery, after a random time (between 180 and 240 seconds) it triggers the battery low signal. It checks if the robot is the E room before waiting the time for the full charge of the robot and then it triggers the battery full status.

*Move_base client*
--
It recieves the coordinates from the *State_machine* and it calls the move base action server, it checks if the robot is `ready_to_move` before starting.

*Joint controller module*
--
At the start of the simulation this module controls in position the rotation of the arm joint of the robot making possible to scan all the markers, the same structure is then repeated inside the `PATROLLING_ROOM` status of the state machine.

*Map module*
--
Is the module in charge of creating the map, it checks if all the markes are detected and creates the map starting from the ontology_map.owl, it creates a new ontoly with all the rooms and the connection between them

*
