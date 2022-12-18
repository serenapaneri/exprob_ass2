#include "exprob_ass2/start_game.h"
#include <unistd.h>
// #include <moveit/move_group_interface/move_group_interface.h>

namespace KCL_rosplan {

    StartGameInterface::StartGameInterface(ros::NodeHandle &nh) {
    // here the initialization
    }
    
    bool StartGameInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        
        // moveit::planning_interface::MoveGroupInterface group("arm");
        // group.setEndEffectorLink("cluedo_link");
        // group.setPoseReferenceFrame("base_link");
        // group.setPlannerId("RRTstar");
        // group.setNumPlanningAttempts(10);
        // group.setPlanningTime(10.0);
        // group.allowReplanning(true);
        // group.setGoalJointTolerance(0.0001);
        // group.setGoalPositionTolerance(0.0001);
        // group.setGoalOrientationTolerance(0.001);
        
        std::cout << "The robot is powering on" << std::endl;
        
        // group.setNamedTarget("default");
	// group.move();
	// sleep(3.0);

        std::cout << "Initializing ARMOR" << std::endl;
        sleep(3);
        std::cout << "Loading the ontology" << std::endl;
        sleep(1);
        std::cout << "Uploading the TBox" << std::endl;
        sleep(1);
        std::cout << "Disjoint the individuals of all classes" << std::endl;
        sleep(1);
        std::cout << "The robot is looking for hints .." << std::endl;
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}

    int main(int argc, char **argv) {
        ros::init(argc, argv, "start_game_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::StartGameInterface start_game(nh);
        start_game.runActionInterface();
        return 0;
    }
