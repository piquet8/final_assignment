# Research Track 1 - final assignment
Develop a software architecture for the control of the robot in the environment. The software will rely on the move_base and gmapping packages for localizing the robot and plan the motion
## Expected Behaviour
The architecture should give the user the possibility to choose which behavior to apply to the robot among the following:
1. move randomly in the environment, by choosing 1 out of 6 possible target positions:
[(-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1)], implementing a random position service as in the assignment 1
2. directly ask the user for the next target position (checking that the position is one of the possible six) 
and reach it
3. start following the external walls
4. stop in the last position
5. (optional) change the planning algorithm to dijkstra (move_base) to the bug0

If the robot is in state 1, or 2, the system should wait until the robot reaches the position in order to switch to the state 3 and 4 or (if implemented) to change the planning algorithm
# Project structure
In this project, 4 nodes (one was already implemented) and a file service were developed. The **python programming language** was used to implement the nodes.
## Node
* [main_node.py](https://github.com/piquet8/final_assignment/blob/main/scripts/main_node.py): it is the main node, in fact it manages all the other nodes and communication with the move base to allow the robot to navigate within the simulation environment
* [user_interface.py](https://github.com/piquet8/final_assignment/blob/main/scripts/user_interface.py): it provides a user interface through which the user can select the type of behaviour to be performed by the robot. It implements the function set_new_state
* [wall_follower_service_m.py](https://github.com/piquet8/final_assignment/blob/main/scripts/wall_follower_service_m.p): it provides a service for simulating the wall following behaviour
* [random_position.py](https://github.com/piquet8/final_assignment/blob/main/scripts/random_position.py): it provides a service to choose randomly the new values of the x and y the coordinates of the new target position
## Messages
### Published messages
* `geometry_msgs/Twist`: it published the messages to the **/cmd_vel** topic. It is used to modify the speed of the robot; in particular, inside the function change_state of the main_node, it has been used to stop the robot by setting its speed to zero.
* `move_base_msgs/MoveBaseActionGoal`: it published the messages to the **/move_bae/goal** topic. Inside the main_node, in the publish_ag function it is used to set the goal that the robot has to reach
* `actionlib_msgs`: it published the messages to the **/move_base/cancel** topic. It is used to remove a target that the robot has reached. It allows to avoid the overlying of robot behaviours
### Subscriber messages
* `geometry_msgs/Point`: it provides the robot position expressed in the **/Point** topic. It is used in the in the main_node 
* `nav_msgs/Odometry`: it provides the current robot position in the **/Odom** topic. It is used in the function target_distance() of the main_node
* `sensor_msgs/LaserScan`: it provides a real-time laser output on the **/scan** topic. It is used in the wall_follower_service
## Services
* Inside of the [random_position.py](https://github.com/piquet8/final_assignment/blob/main/scripts/random_position.py) script there is the **/Random_position.srv** service: it provides a pair of randomly coordinates between the six possible ones. Request/reply interactions are done via a service file [Random_position.srv](https://github.com/piquet8/final_assignment/blob/main/srv/Random_position.srv) which has an empty request field and a two float response fields
* Inside of the [wall_follower_service_m.py](https://github.com/piquet8/final_assignment/blob/main/scripts/wall_follower_service_m.p) script there is the **/wall_follower_switch**: it provides to activate/deactivate the wall_follower behaviour
* Inside of the [user_interface.py](https://github.com/piquet8/final_assignment/blob/main/scripts/user_interface.py) script there is the **/user_interface** service: it provides to ask to the user a possible behaviour between the four possible ones to change the type of navigation of the robot and thus changing the state
## Parameters
- They are inside of the assignment.launch and are:
    - `des_pos_x` and `des_pos_y` for memorize the coordiantes x and y of the target that the robot have to reach
    - `state` for the individuation of the current state of the robot
    - `change_state` for initialising at zero the current state of the robot
## Rqt-graph
Here we can see a dynamic graph showing what's going on in the system:
![Rqt-graph](https://github.com/piquet8/final_assignment/blob/main/Rqt_graph2.png)
# How to launch
1. Firstly, open the terminal, go to your workspace and in the src folder run:
```
git clone https://github.com/piquet8/final_assignment.git
```
2. To launch the simulation 3D environment run the command:
```
roslaunch final_assignment simulation_gmapping.launch
```
3. Then to launch the move_base open a new shell tab and run the command:
```
roslaunch final_assignment move_base.launch
```
**Remember** that python files must be made executable before they are run, to do this, go to the directory of the file and use: `chmod +x file_name.py`

4. Finally to start the navigation with the robot, open a new shell tab and run the command:
```
roslaunch final_assignment assignment.launch
```
# Report of the assignment
## Robot behaviors
Four behaviours have been implemented: move randomly, target position, walls following and stop. In the first two states, the robot can reach six possible target positions (-4,-3);(-4,2);(-4,7);(5,-7);(5,-3);(5,1).
By choosing the **move randomly** behaviour the robot will receive a random target position among the six possible ones and will reach it without user intervention. Choosing the behaviour **target position** the user will have to choose manually which of the six destinations to reach. In both cases it will not be possible to change the behaviour until the robot has reached the target position. The third behaviour is **walls following**, as the name suggests the robot moves within the environment following the walls it finds nearby. Finally, there is the last behaviour **stop**, it stops the robot.  
## Software architecture and the architectural choices made
The project architecture consists of four nodes: [main_node.py](https://github.com/piquet8/final_assignment/blob/main/scripts/main_node.py), [user_interface.py](https://github.com/piquet8/final_assignment/blob/main/scripts/user_interface.py), [random_position.py](https://github.com/piquet8/final_assignment/blob/main/scripts/random_position.py) and [wall_follower_service_m.py](https://github.com/piquet8/final_assignment/blob/main/scripts/wall_follower_service_m.p).
The first one manages the other three, they are called in its script as services to implement the different functions. 

In the [user_interface.py](https://github.com/piquet8/final_assignment/blob/main/scripts/user_interface.py) node the function `set_new_state()` is implemented, which asks the user to choose one of the four possible behaviours and, after checking the suitability of the choice, communicates it to the main_node setting the new state. In the [random_position.py](https://github.com/piquet8/final_assignment/blob/main/scripts/random_position.py) node a simple function is implemented which determines the new target among the possible six; a random number between 1 and 6 determines which of the six targets is the choice and sent it via reply/request service with file [Random_position.srv](https://github.com/piquet8/final_assignment/blob/main/srv/Random_position.srv) to the main node. The [wall_follower_service_m.py](https://github.com/piquet8/final_assignment/blob/main/scripts/wall_follower_service_m.p) node is an already implemented node that allows the robot to navigate in the environment by following the walls that surround it.

Inside the [main_node.py](https://github.com/piquet8/final_assignment/blob/main/scripts/main_node.py) there are three main functions: the `target_distance()` function that controls the distance between the robot and the target position allowing to determine when the robot reaches the target, the `publish_ag()` function that communicates with the move_base setting the position to be reached and finally the `change_state()` function that allows to change from one state to another. The latter function contains four states: in state 1 the [main_node.py](https://github.com/piquet8/final_assignment/blob/main/scripts/main_node.py) asks the [random_position.py](https://github.com/piquet8/final_assignment/blob/main/scripts/random_position.py) node for a random position among the six possible, in state 2 the user is asked to choose one of the six possible positions and after check the numbers chosen it sets the new target. In state 3 the [wall_follower_service_m.py](https://github.com/piquet8/final_assignment/blob/main/scripts/wall_follower_service_m.p) node is called so that the robot can navigate inside the environment following the walls, finally in state 4 the speed is set to zero and the robot receives the stop command. At the end of state 3 and 4 the [user_interface.py](https://github.com/piquet8/final_assignment/blob/main/scripts/user_interface.py) node is called so that the user can select a new behaviour. In cases 1 and 2, this call is made in the main() only after the target position has been reached.
## System’s limitations and possible improvements
The use of the bug algorithm could also have been added, I tried to do this but after activating it and typing in a target position the robot tried to reach it but did not return the message of reaching or not reaching it, in fact it went straight to asking to enter a new status. In addition, the map seen on srviz during navigation with the bug algorithm was distorted, causing problems in subsequent navigations.

A good improvement could be to modify the speed of the robot in relation to its surroundings, so that for example it is faster in sections without obstacles or curves; this is because when for example the robot passes from a very high position (e.g. -4,7) to a very low one (e.g. 5,-7) it takes a long time
