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
## Node
*   
* 
* [wall_follower_service_m.py](https://github.com/piquet8/final_assignment/blob/main/scripts/wall_follower_service_m.p): it provides a service for simulating the wall following behaviour
* [random_position.py](https://github.com/piquet8/final_assignment/blob/main/scripts/random_position.py): it provides a service to choose a new target position randomly
## Rqt-graph
Here we can see a dynamic graph showing what's going on in the system:
![Rqt-graph](https://github.com/piquet8/final_assignment/blob/main/Rqt_graph2.png)


