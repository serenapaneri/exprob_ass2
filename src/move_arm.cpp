/** @package exprob_ass2
*
* \file move_arm.cpp
* \brief this node implements the move_arm action.
*
* \author Serena Paneri
* \version 1.0
* \date 5/1/2023
*
* \details
*
* Subscribes to: <BR>
*     None
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
* Action Client: <BR>
*     None
*
* Description: <BR>
* In this node is implemented the move_arm action of the rosplan.
* This node makes the arm of the robot robot move in a high or a low pose, depending on the value
* retrieved by the parameter server, that way previously set in the start_game action.
*/

#include "exprob_ass2/move_arm.h"
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>

// all the z coordinates of the four waypoints
double marker_z1 = 0.0;
double marker_z2 = 0.0;
double marker_z3 = 0.0;
double marker_z4 = 0.0;

namespace KCL_rosplan {

    MoveArmInterface::MoveArmInterface(ros::NodeHandle &nh) {
        // here the initialization
    }


/**
* \brief Callback of the move_arm action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action move_arm is executed. This action allows
* the robot its own arm in the high or low pose, implemented with moveit, depending on the z retrived by the
* parameter server.
*/    
    bool MoveArmInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "The robot is trying to collect an hint" << std::endl;
        
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
        
        if(msg->parameters[1].value == "wp1"){
            ros::param::get("wp1", marker_z1);
            if (marker_z1 == 0.75) {
                group.setNamedTarget("low");
	        group.move();
	        sleep(3.0);
            }
            else if (marker_z1 == 1.25) {
                group.setNamedTarget("high");
	        group.move();
	        sleep(3.0);
            }
        }
        
        else if(msg->parameters[1].value == "wp2"){
            ros::param::get("wp2", marker_z2);
            if (marker_z2 == 0.75) {
                group.setNamedTarget("low");
	        group.move();
	        sleep(3.0);
            }
            else if (marker_z2 == 1.25) {
                group.setNamedTarget("high");
	        group.move();
	        sleep(3.0);
            }
        }
        
        else if(msg->parameters[1].value == "wp3"){
            ros::param::get("wp3", marker_z3);
            if (marker_z3 == 0.75) {
                group.setNamedTarget("low");
	        group.move();
	        sleep(3.0);
            }
            else if (marker_z3 == 1.25) {
                group.setNamedTarget("high");
	        group.move();
	        sleep(3.0);
            }
        }
        
        else if(msg->parameters[1].value == "wp4"){
            ros::param::get("wp4", marker_z4);
            if (marker_z4 == 0.75) {
                group.setNamedTarget("low");
	        group.move();
	        sleep(3.0);
            }
            else if (marker_z4 == 1.25) {
                group.setNamedTarget("high");
	        group.move();
	        sleep(3.0);
            }
        } 
	
	    
	std::cout << "A new hint has been collected!" << std::endl;
        

        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}


/**
* \brief Main function of the move_arm action. 
* \param None
* \return 0
*
* This is the main function of the move_arm action, where the node is initialized. Moreover there is the 
* MoveArmInterface to execute the real action as an action of the rosplan.
*/    
    int main(int argc, char **argv) {
        ros::init(argc, argv, "move_arm_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::MoveArmInterface move_arm(nh);
        move_arm.runActionInterface();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        sleep(2.0);
        return 0;
    }

