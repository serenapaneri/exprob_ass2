/** @package exprob_ass2
*
* \file start_game.cpp
* \brief this node implements the start_game action.
*
* \author Serena Paneri
* \version 1.0
* \date 5/1/2023
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
* Action Client: <BR>
*     /go_to_point
*
* Description: <BR>
* In this node is implemented the first action of the plan when the simulation starts.
* This node simply perform a complete first tour moving the arm both in the high position and then in the low
* position. This is done only to know the z poistion of the sphere in the environment, in a way that, in the
* following rounds the robot already knows the position of those spheres and positions directly in the right
* position. 
*/

#include "exprob_ass2/start_game.h"
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>
#include <exprob_ass2/ErlOracle.h>

// subscriber of the topic oracle_hint
ros::Subscriber hint_sub;

// callback of the oracle_hint subscriber
void hint_callback(const exprob_ass2::ErlOracle::ConstPtr& msg);
// function to move the robot
void move_to(double xpos, double ypos, double orientation);
// function to adjust the robotic arm in the default pose
int default_pose();
// function to adjust the robotic arm in the high pose
int high_pose();
// function to adjust the robotic arm in the low pose
int low_pose();

// markers position
double marker_z1 = 0.0;
double marker_z2 = 0.0;
double marker_z3 = 0.0;
double marker_z4 = 0.0;

bool collected = false;

namespace KCL_rosplan {

    StartGameInterface::StartGameInterface(ros::NodeHandle &nh) {
    // here the initialization
    }
    
    
/**
* \brief Callback of the start_game action
* \param msg: message from the plan_dispatcher
* \return true
*
* This function implements the beahvior of the robot when the action start_game is executed. In particular
* this action is only execute once at the beginning of the simulation, with the purpose of store in the 
* parameter server the z position of the sphere in the environment.
*/
    bool StartGameInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        
        std::cout << "The robot is powering on" << std::endl;
        
        // default pose of the arm
        default_pose();

        std::cout << "Initializing ARMOR" << std::endl;
        sleep(3);
        std::cout << "Loading the ontology" << std::endl;
        sleep(1);
        std::cout << "Uploading the TBox" << std::endl;
        sleep(1);
        std::cout << "Disjoint the individuals of all classes" << std::endl;
        sleep(1);
        std::cout << "Start visiting all the waypoints" << std::endl;
        
        // move to the first waypoint 
        move_to(2.2, 0.0, 0.0);
        
        // high pose of the arm
        high_pose();
        
        // if the robot has collectd the hint in the high position
        if(collected == true){
            marker_z1 = 1.25;
        }
        // otherwise it will be collected in the low position
        else {
            marker_z1 = 0.75;
        }
        
        // set the z position of the marker with a parameter server
        ros::param::set("wp1", marker_z1);
        
        // low pose of the arm
        low_pose();
        
        // default pose of the arm
        default_pose(); 
        
        collected = false;
        
        std::cout << "The marker at wp1 is at: " << marker_z1 << std::endl;
        
        // move to the second waypoint 
        move_to(0.0, 2.2, 1.57);
        
        // high pose of the arm
        high_pose();
        
        // if the robot has collectd the hint in the high position
        if(collected == true){
            marker_z2 = 1.25;
        }
        // otherwise it will be collected in the low position
        else {
            marker_z2 = 0.75;
        }
        
        // set the z position of the marker with a parameter server
        ros::param::set("wp2", marker_z2);
        
        // low pose of the arm
        low_pose();
        
        // default pose of the arm
        default_pose();
        
        collected = false;
        
        std::cout << "The marker at wp2 is at: " << marker_z2 << std::endl;
        
        // move to the third waypoint 
        move_to(- 2.2, 0.0, 3.14);
        
        // high pose of the arm
        high_pose();
        
        // if the robot has collectd the hint in the high position
        if(collected == true){
            marker_z3 = 1.25;
        }
        // otherwise it will be collected in the low position
        else {
            marker_z3 = 0.75;
        }
        
        // set the z position of the marker with a parameter server
        ros::param::set("wp3", marker_z3);
        
        // low pose of the arm
        low_pose();
        
        // default pose of the arm
        default_pose();
        
        collected = false;
        
        std::cout << "The marker at wp3 is at: " << marker_z3 << std::endl;
        
        // move to the first waypoint 
        move_to(0.0, - 2.2, -1.57);
        
        // high pose of the arm
        high_pose();
        
        // if the robot has collectd the hint in the high position
        if(collected == true){
            marker_z4 = 1.25;
        }
        // otherwise it will be collected in the low position
        else {
            marker_z4 = 0.75;
        }
        
        // set the z position of the marker with a parameter server
        ros::param::set("wp4", marker_z4);
        
        // low pose of the arm
        low_pose();
        
        // default pose of the arm
        default_pose();
        
        collected = false;
        
        std::cout << "The marker at wp4 is at: " << marker_z4 << std::endl;
        
        // return home
        move_to(0.0, 0.0, 0.0);
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}


/**
* \brief Main function of the start_game action. 
* \param None
* \return 0
*
* This is the main function of the start game_action, where the node is initialized and the subscriber of the
* oracle_hint is implemented. Moreover there is the StartGameInterface to execute the real action as an action
* of the rosplan.
*/
    int main(int argc, char **argv) {
    
        ros::init(argc, argv, "start_game_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");

        hint_sub = nh.subscribe("/oracle_hint", 1000, hint_callback);
        
        KCL_rosplan::StartGameInterface start_game(nh);
        start_game.runActionInterface();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        sleep(1.0);
        return 0;
    }


/**
* \brief Callback function of the subscriber to the /oracle_hint topic. 
* \param msg: message that contains the hint
* \return None
*
* This is the callback function of the subscriber to the /oracle_hint topic. In this case the 
* message is not collected but a variable is set to true when a message is recieved. 
*/
void hint_callback(const exprob_ass2::ErlOracle::ConstPtr& msg){

    collected = true;
}


/**
* \brief Function to move the arm in the default position. 
* \param None
* \return 0
*
* This function uses the output of the moveit setup assistant in order to set the robotic
* arm into the default position, that is a pose created with moveit. 
*/   
int default_pose() {

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
    group.setNamedTarget("default");
    group.move();
    sleep(3.0);
    return 0;
    
}


/**
* \brief Function to move the arm in the high position. 
* \param None
* \return 0
*
* This function uses the output of the moveit setup assistant in order to set the robotic
* arm into the high position, that is a pose created with moveit. 
*/ 
int high_pose() {

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
    group.setNamedTarget("high");
    group.move();
    sleep(3.0);
    return 0;
    
}


/**
* \brief Function to move the arm in the low position. 
* \param None
* \return 0
*
* This function uses the output of the moveit setup assistant in order to set the robotic
* arm into the low position, that is a pose created with moveit. 
*/ 
int low_pose() {

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
    group.setNamedTarget("low");
    group.move();
    sleep(3.0);
    return 0;
    
}


/**
* \brief Function to make the robot move in the enviornment. 
* \param xpos: x position in the environment
* \param ypos: y position in the environment
* \param orientation: desired orientation in the environment
* \return None
*
* This function implement an action client in order to make the robot move within the 
* environment that allows also to stop immediatly the behavior of the robot when this
* node is stopped. 
*/ 
void move_to(double xpos, double ypos, double orientation) {

    actionlib::SimpleActionClient<exprob_ass2::TargetAction> ac("/go_to_point", true);
    exprob_ass2::TargetGoal goal;
    ac.waitForServer();
    goal.x = xpos;
    goal.y = ypos;
    goal.theta = orientation;
    ac.sendGoal(goal);
    ac.waitForResult();
    
}
