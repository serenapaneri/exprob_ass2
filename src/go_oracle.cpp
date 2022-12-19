#include "exprob_ass2/go_oracle.h"
#include <unistd.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>

namespace KCL_rosplan {

    GoOracleInterface::GoOracleInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool GoOracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        std::cout << "The robot is going to the " << msg->parameters[2].value << " room from " << msg->parameters[1].value << std::endl;
        
        actionlib::SimpleActionClient<exprob_ass2::TargetAction> ac("/go_to_point", true);
        exprob_ass2::TargetGoal goal;
        ac.waitForServer();
        
        goal.x = 2.5;
        goal.y = - 2.5;
        goal.theta = - M_PI/4;
        
        ac.sendGoal(goal);
        ac.waitForResult();
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}

    int main(int argc, char **argv) {
        ros::init(argc, argv, "go_oracle_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::GoOracleInterface go_oracle(nh);
        go_oracle.runActionInterface();
        return 0;
    }
