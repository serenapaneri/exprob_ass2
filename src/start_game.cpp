#include "exprob_ass2/start_game.h"
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>
#include <exprob_ass2/ErlOracle.h>

ros::Subscriber hint_sub;

void hint_callback(const exprob_ass2::ErlOracle::ConstPtr& msg);
void move_to(double xpos, double ypos, double orientation);
int default_pose();
int high_pose();
int low_pose();

double marker_z1 = 0.0;
double marker_z2 = 0.0;
double marker_z3 = 0.0;
double marker_z4 = 0.0;

bool collected = false;

namespace KCL_rosplan {

    StartGameInterface::StartGameInterface(ros::NodeHandle &nh) {
    // here the initialization
    }
    
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

 
void hint_callback(const exprob_ass2::ErlOracle::ConstPtr& msg){
    collected = true;
}
    
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
