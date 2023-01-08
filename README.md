# Second Assignment of the Experimental Robotics Laboratory course (Robotics Engineering/ JEMARO, Unige)

## Brief introduction
This ROS package contains the implementation of the second assignment of the Experimental Robotics Laboratory course. 
The aim of this project is to implement a vanilla version of the cluedo game and all the procedure can be seen thanks to a simulation environment both in gazebo and rviz.
Indeed during the execution of the simulation there is a robot that navigates between 6 different point of the environment, which four of them are waypoints in which the robot can collect hints moving its own arm, and the other two points are the home and the oracle room.
The purpose of that simulation is, just as in the game, the search for a murderer, the murder weapon and the crime scene.
Indeed the robot navigates within these four waypoint with the purpose of collecting clues to solve the mistery. Indeed, collecting these hints, it is able to formulate hypotheses, thus trying to find the winning one that will be revealed by the oracle of the game in the oracle room. 
It was asked also to model the robot and create some poses to better reach the hints (that can be found in floating spheres at two different heights randomly chosen) using the moveit setup assistant, and to implement the behavioral software architecture of the project. To menage all the knowledge concerning the hints and the hypotheses, an ontology has been used.

## Robot model
![Alt text](/images/rviz_robot.png?raw=true)
![Alt text](/images/gazebo_robot.png?raw=true)

The model of the robot is contained in the two files, contained in the urdf folder, [**cluedo_robot.gazebo**](https://github.com/serenapaneri/exprob_ass2/tree/main/urdf/cluedo_robot.gazebo) and [**cluedo_robot.xacro**](https://github.com/serenapaneri/exprob_ass2/tree/main/urdf/cluedo_robot.xacro). Moreover in the same folder you can find the [**materials.xacro**](https://github.com/serenapaneri/exprob_ass2/tree/main/urdf/materials.xacro), where the different materials that can be used in the robot are created, and the [**cluedo_robot.urdf**](https://github.com/serenapaneri/exprob_ass2/tree/main/urdf/cluedo_robot.urdf) that is the file automatically generated from the moveit package that is used to create poses for your robot model. 
The overall structure of the robot model is descripted below:

![Alt text](/images/cluedo_robot_graphix.png?raw=true) 

## Moveit

It is a robotic manipulation platform that allows to develop manipulation application. Thanks to the moveit setup assistant three different poses for the robotics arm have been developed:

- Default pose

![Alt text](/images/defaultpose.png?raw=true)

This is the default pose in which the robotic arm is found during the execution of the game, except when it needs to collect an hint. In this last case the robot would assume one of the following pose, depending if the floating sphere is positioned at a height of 1.25 or 0.75 

- High pose

![Alt text](/images/highpose.png?raw=true)

This is the pose assumend by the robot when the floating sphere is located at 1.25.

- Low pose

![Alt text](/images/lowpose.png?raw=true)

This is the pose assumend by the robot when the floating sphere is located at 0.75.


## Software architecture
### PDDL
The execution of the whole package is handled by the ROSPLAN, that allows to execute the various action described in the [**domain**](https://github.com/serenapaneri/exprob_ass2/tree/main/common/domain.pddl) file, and then implemented as c++ nodes, starting from the init condition contained in the [**problem**](https://github.com/serenapaneri/exprob_ass2/tree/main/common/problem.pddl) file, which also contains the goal of the plan. The actions written as c++ nodes are described in the following section. 

### Component diagram

![Alt text](/images/componentdiagram.png?raw=true)

With the component diagram it is possible to see the overall behavior and how the whole architecture is organized.
In this diagrams are shown, besides the armor service, the four nodes of which the package is composed.

#### Python nodes

- [**menage_ontology**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/menage_ontology.py): In this node are defined all the list of individuals, divided in people, weapons and places.
This node contains a client for the ARMOR service, to be able to menage the cluedo_ontology.
In particular, the operation performed, thanks to this service, are the loading of the cluedo_ontology, the upload of the TBox of the game containing all the individuals, the disjoint of the individuals belonging to each class and the starting of the reasoner of the ontology.

- [**hints**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/hints.py): In this node are handled the client of the ARMOR service, the hint subscriber, the complete service and the hypo_found service. 
This node is in charge of menaging the flow of hints recieved, also evaluating, and in this case discard those, if the hints recieved are malformed. In this case of course the hints will not be uploaded in the ontology either. This node it is also in charge of checking the completeness of the current hypotheses that have been found till that moment.
Everytime an hint is recieved, so at every waypoint, the completeness is evaluated, and if there are not new complete hypothesis than the node plan_execution.py will start a replanning. If instead a new complete hypothesis is found then it keeps going with the current plan. 
There is also the reasoner and the apply function, that are services provided by the ARMOR package, that allows to directly interact with the ontology, in order to apply changes and update the knowledge everytime a new hint is uploaded. 

- [**check_hypothesis**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/check_hypothesis.py): In this node are handled the armor client, the hypo_found_client, the comm_service, the win_hypo_srv and cons_service.
In this node the consistency of the current hypothesis is checked and this is done, first retrieving the ID of the current complete hypothesis from the hypo_found service, and then retrieving the list of object properties of a certain individual (that in this case is the current hypothesis), and evaluating the number of elements that composes that specific hypothesis to see if the hypothesis is consistent or not.
If the hypothesis is complete and consistent the node check_consistency is advertised and those elements are sent to the oracle node, if insead the hypothesis is incosistent, the check_consistency node is advertise in order to perform a replanning.

- [**go_to_point**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/go_to_point.py): In this node is implemented the behavior of the robot when it needs to move to a specific location. Indeed there are functions that allow to robot to rotate and move straight in order to reach a target position and another function to adjust its final orientation. This is also done thanks to an action service that allows the client to stop the behavior of the robot even if when it is in the middle of the execution of this task.

- [**plan_execution**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/plan_execution.py): In this node are called all the service needed, provided by the ROSPLAN, that take the domain and the problem file written with the PDDL formalism, generate a plan based on the "instruction" given from these two files, and when the plan fails for different reasons, it also performs automatically a replan, giving a new brand plan.

#### C++ Nodes
All the c++ nodes corresponds to a specific action of the ROSPLAN, except the simulation node.

- [**simulation**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/simulation.cpp): In this node the hints of the game are generated randomly, taking elements of the lists person, object and place and associating to them an ID. This hints are published on the topic /oracle_hint and they also comprehend malformed hints, that are hints where some field are missing or wrong. In addition, it is also chosen the ID of the winning hypothesis and this values is stored in the /oracle_solution topic. Finally also the setting of the environment is implemented, indeed some floating sphere are published in the environment and, right in these sphere, the robot can percieved the hints of the game. Moreover the arm of the robot is set to the default pose, mimiking the powering on of the robot.

- [**start_game**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/start_game.cpp): This node represents the first action executed by the plan when all the ROSPLAN services are called. This node is execute only once at the beginning of the simulation since the goal of this node is to make a first tour of the waypoints of the environment, at where the floating sphered are located, in order to store the z coordinates of the sphere in a parameter server. Doing that it makes more efficient the search of new hints in the following rounds, since the robot would already know in which pose it should place its arm in order to collect a new hint. 

- [**leave_home**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/leave_home.cpp): In this node is implemented an action client that allows the robot to move from the home, that is positioned at the centre of the simulation environment, to one of the four waypoints in the game. This node can be executed at the beginning of the simulation, after the start_game action, or when the robot find an hypothesis that is inconsistent and needs to continue its research of a complete and consistent hypothesis collecting more hints.

- [**go_to_waypoint**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/go_to_waypoint.cpp): In this node is implemented an action client that allows the robot to move from a waypoints to another one in order to collect hints. And this action is repeatetd until the robot finds a complete hypothesis and can go then to home to check the consistency of the new complete hypothesis just found. Moreover at the beginning of this action the robot arm is set to the default pose from the high or low one, set in the previous action.

- [**move_arm**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/move_arm.cpp): In this node it is performed the motion of the arm when the robot is in a waypoint of the simulation environment. The z coordinates, previously store in the start_game action are retrieved, in a way that the robot already know which pose of the arm to use to reach the floating sphere in order to collect an hint. The to avaiable poses done with the moveit setup assistant are the high pose, for reaching the floating sphere with a height of 1.25, and the low pose, for reaching the floating sphere with height 0.75.

- [**check_complete**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/check_complete.cpp): In this node is checked if a complete hypothesis has been found or not. This action is executed at every waypoint, indeed if a new complete hypothesis is found then the plan can proceed, instead if no new complete hypotheses have been found then this action can fail, causing a replanning.

- [**go_home**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/go_home.cpp): In this node is implemented an action client that allows the robot to move from one of the waypoints to home, which is located at the centre of the simulation environment. This action is executed only when a new complete hypothesis has been found and the robot, according to the current plan, needs to go home to check the consistency of that hypothesis.

- [**check_consistency**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/check_consistency.cpp): In this node is checked if the last complete hypothesis found is also consistent, this action is performed at home. If the hypothesis just found is complete and consistent, the current plan can proceed with its execution, if instead the hypothesis is inconsistent, this will cause a failure in the action and a replanning is performed.

- [**go_oracle**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/go_oracle.cpp): In this node is implemented an action client that allows the robot to move from home, that is at the center of the simulation environment, to the oracle room, that is located in the lower right corner of the simulation environment. This action is performed only when a complete and consistent hypothesis has been found. 

- [**oracle**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/oracle.cpp): In this node is checked if the current complete and consistend hypothesis is the winning one. This is done by comparing the ID of the winning hypothesis with the ID of the current hypothesis. If the two IDs matches, then the plan is fully executed and the game is finished, if not the action would fail and there would be a replanning.

- [**leave_oracle**](https://github.com/serenapaneri/exprob_ass2/tree/main/src/leave_oracle.cpp): In this node is implemented an action client that allows the robot to move from the oracle room to one of the four waypoints of the simulation environment. This action is performed only when the last complete and consistent hypothesis found is not the winning one, to allow the robot to continue its investigation collecting new hints. 

### State diagram

![Alt text](/images/statediagram.png?raw=true)

In this repository is not implemented a real state machine, but the execution of the various nodes, which for the most part represent actions, is handled by the ROSPLAN. All those nodes are executing calling four main services of the rosplan, that are recursively called by the plan_execution node, that are: 

- /rosplan_problem_interface/problem_generation_server: It is used to generate a problem instance.
- /rosplan_planner_interface/planning_server: It takes the domain file and the problem instance in order to genrate a plan. 
- /rosplan_parsing_interface/parse_plan: It is used to convert the planner output into a plan representation that can be executed. 
- /rosplan_plan_dispatcher/dispatch_plan: It is used to execute the plan.

When the dispatch plan return false, meaning that the plan failed for some reason, all the services are called again in order to see the actual state of the knowledge base and create, based on that state, a new plan that will be executed until the plan fails again or it completes its execution. 
In this case only three action of the plan can cause a failure, and thus the need of a replanning. These action are:
- check_complete: That is false until a new complete hypothesis is found.
- check_consistency : That returns true, thus the action succedes, when the current complete hypothesis is also consistent, otherwise it returns false causing a replanning.
- oracle : If the ID of the current complete and consistent hypothesis and the ID of the winning one coincides then the action returns true and the plan finished its execution, if not it returns false causing a replanning.

### Messages, Services and RosParameters
#### Messages
In the msg folder you can find the [**ErlOracle.msg**](https://github.com/serenapaneri/exprob_ass2/tree/main/msg/ErlOracle.msg) message.
The message, that os sent thanks to the hint publisher in the simulation onde, has the following structure:
> int32 ID

> string key

> string value

So, the message contains the ID associated to the hint, the type of hint that can be 'who', 'what' and 'where', and the individual of that hint.

#### Services
In the srv folder you can find different services:
- [**Command.srv**](https://github.com/serenapaneri/exprob_ass2/tree/main/srv/Command.srv): The structure is the one below:

**Request**
> string command

**Response**
> bool answer

So, it contains the request done by the client that is a string, and the response is a bool.
- [**HypoFound.srv**](https://github.com/serenapaneri/exprob_ass2/tree/main/srv/HypoFound.srv): The structure is the one below:

**Response**
> int32 IDs

It only contains the response to the client that is an integer and is used to send the ID of the current complete hypothesis just found.
- [**Oracle.srv**](https://github.com/serenapaneri/exprob_ass2/tree/main/srv/Oracle.srv): The structure is the one below:

**Response**
> int32 ID

It only contains the response to the client that is an integer and is used to send the ID of the winning hypothesis.
- [**Complete.srv**](https://github.com/serenapaneri/exprob_ass2/tree/main/srv/Complete.srv): The structure is the one below:

**Response**
> bool complete

It only contains the response to the client and it is a bool that is set to true only when a new complete hypothesis is found. 
- [**Consistent.srv**](https://github.com/serenapaneri/exprob_ass2/tree/main/srv/Consistent.srv): The structure is the one below:

**Response**
> bool consistent

It only contains the response to the client and it is a bool that is set to true only when, after checking the consistency, the hypothesis is complete and consistent. 
- [**WinHypo.srv**](https://github.com/serenapaneri/exprob_ass2/tree/main/srv/WinHypo.srv): The structure is the one below:

**Response**
> string who

> string what

> string where

It only contains the response to the client and here are sent all the field of the hypothesis after checking that that hypothesis is complete and consistent.

#### RosParameters
Within the code, you can notice the use of the ROS parameter server. In this case it has been used to store, at the beginning of the simulation, in particular in the action start_game, the z coordinates of the floating sphere. These values are then retrieved by the move_arm action in the following rounds. This is done to mimic a first inspection of the simulation environment in the first tour, and then, retrieving those parameters, the robot would already know where those sphere are positioned and it can easily reach them using only one pose. This increases the efficiency and reduce the time needed to solve the mistery, which is already time consuming.

## Installation and running procedures
### Installation
To install the package you just need to clone the repository by typing in the terminal:
```
  git clone https://github.com/serenapaneri/exprob_ass2.git
```
Remember that you also need to clone the package generated with the moveit setup assistant in the branch "moveit", by typing in the terminal:
```
  git clone -b moveit https://github.com/serenapaneri/exprob_ass2.git
```
and then simply run the catkin make in your ROS workspace:
```
  catkin_make
```
Before executing the project you should install, if they are not already installed, the following packages:
- [**ARMOR**](https://github.com/EmaroLab/armor.git)
- [**MOVEIT**](https://github.com/ros-planning/moveit)
- [**ROSPLAN**](https://github.com/KCL-Planning/ROSPlan)
### Running procedure
After you complete the step aforementioned, you can finally run the whole program by typing in the terminal:
```
  roslaunch exprob_ass2 cluedo_robot_moveit.launch 1>/dev/null 2>/dev/null
```
With the [**cluedo_robot_moveit.launch**](https://github.com/serenapaneri/exprob_ass2/tree/main/launch/cluedo_robot_moveit.launch) you will launch the simulation environment both in rviz and gazebo, spawning also the robot model. The additional instrunction " 1>/dev/null 2>/dev/null " is used only to not show the warning errors of the robot model.
```
  roslaunch exprob_ass2 rosplan.launch
```
With the [**rosplan.launch**](https://github.com/serenapaneri/exprob_ass2/tree/main/launch/rosplan.launch) you will launch the ROSPLAN interface with all its services.
```
  roslaunch exprob_ass2 plan_services.launch
```
With the [**plan_services.launch**](https://github.com/serenapaneri/exprob_ass2/tree/main/launch/plan_services.launch) you will launch the armor service and the pyhton nodes needed for the simulation.
```
  rosrun exprob_ass2 plan_execution.py
```
With the [**plan_execution.py**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/plan_execution.py) you will launch all the rosplan services needed for starting the simulation and, when needed, the replanning.

### Display robot's behavior
If you also want to visualise the current plan, you can use the command:
```
  rostopic echo /rosplan_planner_interface/planner_output -p
```
## Behavior of the package
The expected behavior of the robot is executed thanks to the ROSPLAN and it is the following:

![Alt text](/images/rosgraph.png?raw=true)

The robot starts its inspection making a first tour of the simulation environment, exploing the various waypoint. At each waypoint the robot moves its arm both in the high pose and in the low pose, since the floating balls, from which the robot can collect the hint, are randomly spawned at the beginning of the simulation at a height that could be 0.75 or 1.25. After collecting the hint, the height of the floating ball at the specific waypoint, is store in a parameter server, in order to be retrieved in the following rounds. This behavior can be clearly seen in the following video:

https://www.youtube.com/embed/_IgGGL0txHw

After that, at each waypoint, the robot compute the consistency. This is done at each waypoint in  order to make the research more efficient, and when a new complete hypothesis has been found, the robot can go home and check also the consistecy. If the current hypothesis is both complete and consistent, then the robot goes to the oracle room to test its hypothesis and see if it is the correct one.
In this plan there are three action that can lead to a replanning, so these actions can fail, and those actions are, check_complete, check_consistency and oracle. Of course, since check_complete is computed at each waypoint, is the one that can fail the most, indeed at almost every waypoits, a replanning is needed.
In the following are shown all the possible scenatios:

- Replanning because check_complete action failed
![Alt text](/images/replanningcomplete.png?raw=true)
 
Since the robot did not find any complete hypothesis, the replanning is executed.
 
- Replanning because check_consistent action failed
![Alt text](/images/replanconsistency.png?raw=true)

Since the current hypothesis was complete but inconsistent, the replanning is executed.

- Replanning because oracle action failed
![Alt text](/images/uncorrecthypothesis.png?raw=true)

Since the current complete and consistent hypothesis was not the winning one, the replanning is executed.

- The plan finished correctly
![Alt text](/images/winninghypo.png?raw=true)

Since the current complete and consistent hypothesis was also the winning one, the plan finished correctly.

## Working hypothesis and environment
The simulation environment with the floating spheres was entirely provided by the Professor, as well as the mechanism for publishing the hints and the oracle service that store the ID of the winning hypothesis. No changes have been made to these files.

### System's features
The whole implementation can be easily changed in order to possible adaptation in future assignments. This is also possbile thanks to its modularity of the program, since each node has a stand alone implementation. 
The fact that the robot makes a first tour for storing the z coordinates of the floating spheres makes the investigation much more efficient in the following rounds, since the robot already know which pose to use to reach the hint.
Also the fact that the robot navigates around waypoints and goes home only when a new complete hypothesis arises, makes everything faster and more efficient.
Moreover when the game is finished, the used can keep track af all the execution of the game by looking at the new ontology generated with inferences.

### System's limitations
A sistem limitation could be the fact that, even when an hypothesis has been classified as complete, and so the hints with the same ID that are no longer considered, they are in any case uploaded in the ontology. Doing that, when the game finished, after finding a complete and consistent hypohesis which is also the winning one, the other hypotheses previously evaluated that were maybe complete and consistent, but not the winning ones, result as inconsistent looking at the ontology with inferences.

### Possible technical improvements
Since each time a new complete hypothesis arises, it is stored, so that it is no longer evaluated later as a new complete hypothesis, it would be good to modify the simulation.cpp node, so that, hints with the same ID of hypotheses whose completeness has already been evaluated, are no longer generated, also because, in any case, they are no longer considered. This would make the duration of the investigation much shorter and more efficient.

## Author and contact 
Serena Paneri, 4506977

s4506977@studenti.unige.it
