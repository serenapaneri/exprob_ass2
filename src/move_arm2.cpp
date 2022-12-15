/** @package exprob_ass2
*
* \file simulation.cpp
* \brief this node implements
*
* \author Serena Paneri
* \version 1.0
* \date 26/11/2022
*
* \details
*
* Subscribes to: <BR>
*     /oracle_hint
*
* Publishes to: <BR>
*     None
*
* Serivces: <BR>
*     None
*
* Client Services: <BR>
*     None
*
* Action Services: <BR>
*     None
*
* Description: <BR>
* In this node the robot executes the motion of its own arm
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

/**
* \brief main function of the node
* \param None 
* \return 0
*
* This is the main function in which the node is initialized 
*/
int main(int argc, char** argv)
{
    // initializing the node
    ros::init(argc, argv, "move_arm2");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);
  
    // moveit
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setEndEffectorLink("cluedo_link");
    group.setPoseReferenceFrame("base_link");
    group.setPlannerId("RRTstar");
    group.setNumPlanningAttempts(10);
    group.setPlanningTime(10.0);
    group.allowReplanning(true);
    group.setGoalJointTolerance(0.0001);
    group.setGoalPositionTolerance(0.0001);
    group.setGoalOrientationTolerance(0.001);
    int choice;
  
    while(ros::ok()){
  
        std::cout << "Press 1 to reach the zero position, press 2 to reach a different position";
        std::cin >> choice;
  
        if (choice==0){
	    group.setNamedTarget("default");
	    group.move();
	    sleep(5.0);
        }
        else if (choice==1){
            group.setNamedTarget("low");
	    group.move();
	    sleep(5.0);
        }
        else if (choice==2){
            group.setNamedTarget("high");
	    group.move();
	    sleep(5.0);
        }
  }
  ros::shutdown();
  return 0;
}
