#include "exprob_ass2/go_home.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace KCL_rosplan {

    GoHomeInterface::GoHomeInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool GoHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        
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
        
        std::cout << "The robot is going " << msg->parameters[2].value << " from " << msg->parameters[1].value << std::endl;
        
        actionlib::SimpleActionClient<exprob_ass2::TargetAction> ac("/go_to_point", true);
        exprob_ass2::TargetGoal goal;
        ac.waitForServer();

        goal.x = 0.0;
        goal.y = 0.0;
        goal.theta = 0.0;
        
        ac.sendGoal(goal);
        ac.waitForResult();
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
        
    }
}


    int main(int argc, char **argv) {
        ros::init(argc, argv, "go_home_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::GoHomeInterface go_home(nh);
        go_home.runActionInterface();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        sleep(2.0);
        return 0;
    }
