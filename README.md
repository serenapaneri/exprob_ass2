# Second Assignment of the Experimental Robotics Laboratory course (Robotics Engineering/ JEMARO, Unige)

## Brief introduction
This ROS package contains the implementation of the second assignment of the Experimental Robotics Laboratory course. 
The aim of this project is to implement a vanilla version of the cluedo game with and all the procedure can be seen thanks to a simulation environment both in gazebo and rviz.
Indeed during the execution of the simulation there is a robot that navigates between 6 different point of the environment, which four of them are waypoints in which the robot can collect hints moving its own arm, and the other two points are the home and the oracle room.
The purpose of that simulation is, just as in the game, the search for a murderer, the murder weapon and the crime scene.
Indeed the robot navigates within these four waypoint with the purpose of collecting clues to solve the mistery. Indeed, collecting these hints, it is able to formulate ehypotheses, thus trying to find the winning one that will be revealed by the oracle of the game in the oracle room. 
It was asked also to model the robot and create some poses to better reaach the hints (that can be found in floating spheres at two different heights randomly chosen) using the moveit setup assistant, and to implement the behavioral software architecture of the project. To menage all the knowledge concerning the hints and the hypotheses, an ontology has been used.

## Software architecture
### Component diagram

HERE GOES THE DIAGRAM

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

- [**simulation**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/simulation.cpp): In this node the hints of the game are generated randomly, taking elements of the lists person, object and place and associating to them an ID. This hints are published on the topic /oracle_hint and they also comprehend malformed hints, that are hints where some field are missing or wrong. In addition, it is also chosen the ID of the winning hypothesis and this values is stored in the /oracle_solution topic. Finally also the setting of the environment is implemented, indeed some floating sphere are published in the environment and, right in these sphere, the robot can percieved the hints of the game.

- [**start_game**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/start_game.cpp): This node represents the first action executed by the plan when all the ROSPLAN services are called. This node is execute only once at the beginning of the simulation since the goal of this node is to make a first tour of the waypoints of the environment, at where the floating sphered are located, in order to store the z coordinates of the sphere in a parameter server. Doing that it makes more efficient the search of new hints in the following rounds, since the robot would already know in which pose it should place its arm in order to collect a new hint. 

- [**leave_home**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/leave_home.cpp):

- [**go_to_waypoint**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/go_to_waypoint.cpp):

- [**move_arm**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/move_arm.cpp):

- [**check_complete**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/check_complete.cpp):

- [**go_home**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/go_home.cpp):

- [**check_consistency**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/check_consistency.cpp):

- [**go_oracle**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/go_oracle.cpp):

- [**oracle**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/oracle.cpp):

- [**leave_oracle**](https://github.com/serenapaneri/exprob_ass2/tree/main/scripts/leave_oracle.cpp):
