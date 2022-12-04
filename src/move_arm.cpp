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
#include <exprob_ass2/ErlOracle.h>

void hint_callback(const exprob_ass2::ErlOracle msg)
{
    
}

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
    ros::init(argc, argv, "move_arm");
    ros::NodeHandle nh;
    // subscriber to /oracle_hint
    ros::Subscriber sub = nh.subscribe("/oracle_hint", 1000, hint_callback);
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
  
	 if (choice==1){
	  group.setNamedTarget("zero");
	  group.move();
	  sleep(5.0);
  }
	  else if (choice==2){
	  geometry_msgs::Pose target_pose1;
          target_pose1.orientation.w = 1.0;
          target_pose1.orientation.x = 0.0;
          target_pose1.orientation.y = 0.0;
          target_pose1.orientation.z = 0.0;
          target_pose1.position.x = 0.1;
          target_pose1.position.y = 0.0;
          target_pose1.position.z = 0.2;
          group.setStartStateToCurrentState();
          group.setApproximateJointValueTarget(target_pose1, "arm_link_04");
          moveit::planning_interface::MoveGroupInterface::Plan my_plan;
          group.plan(my_plan);
          group.execute(my_plan);
          sleep(5.0);
  }
  }
  ros::shutdown();
  return 0;
}
