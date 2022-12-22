#include "exprob_ass2/move_arm.h"
#include <unistd.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exprob_ass2/ErlOracle.h>
#include <exprob_ass2/Complete.h>

void hint_callback(const exprob_ass2::ErlOracle msg);
int collected = 0;
bool complete = false;

double marker_z1 = 0.0;
double marker_z2 = 0.0;
double marker_z3 = 0.0;
double marker_z4 = 0.0;

namespace KCL_rosplan {

    MoveArmInterface::MoveArmInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
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
        
        if(msg->parameters[1] == "wp1"){
            ros::param::get("wp1", marker_z1)
        }
        
        else if(msg->parameters[1] == "wp2"){
            ros::param::get("wp2", marker_z2)
        }
        
        else if(msg->parameters[1] == "wp3"){
            ros::param::get("wp3", marker_z3)
        }
        
        else if(msg->parameters[1] == "wp4"){
            ros::param::get("wp4", marker_z4)
        }
        
        group.setNamedTarget("high");
	group.move();
	// sleep(3.0);
	
	std::cout << collected << std::endl;
	
	//if the hint has not been collected try another pose
	// if (collected == 0){
	group.setNamedTarget("low");
	group.move();
	sleep(3.0);
	//    }
	    
	std::cout << "A new hint has been collected!" << std::endl;
        

        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}

    bool complete_hypo(exprob_ass2::Complete::Request &req, exprob_ass2::Complete::Response &res) {
    
        if (req.verify == "complete") {
            complete = true;
            }
        else if (req.verify == "notcomplete") {
            complete = false;
            }
        return complete;
        }
    
    int main(int argc, char **argv) {
        ros::init(argc, argv, "move_arm_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::MoveArmInterface move_arm(nh);
        move_arm.runActionInterface();
        ros::Subscriber hint_sub = nh.subscribe("/oracle_hint", 1000, hint_callback);
        ros::ServiceServer complete_service = nh.advertiseService("complete_hypo", complete_hypo);
        ros::AsyncSpinner spinner(1);
        spinner.start();
        sleep(2.0);
        return 0;
    }
    
    void hint_callback(const exprob_ass2::ErlOracle msg) {
        collected = 1;
        std::cout << collected << std::endl; 
    }
