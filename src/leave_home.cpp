#include "exprob_ass2/leave_home.h"
#include <unistd.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exprob_ass2/TargetAction.h>

namespace KCL_rosplan {

    LeaveHomeInterface::LeaveHomeInterface(ros::NodeHandle &nh) {
        // here the initialization
    }
    
    bool LeaveHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        
        std::cout << msg->parameters[0].value << std::endl;
        std::cout << msg->parameters[1].value << std::endl;
        std::cout << msg->parameters[2].value << std::endl;
        std::cout << "Leaving  " << msg->parameters[1].value << " for going to " << msg->parameters[2].value << std::endl;
        
        actionlib::SimpleActionClient<exprob_ass2::TargetAction> ac("/go_to_point", true);
        exprob_ass2::TargetGoal goal;
        ac.waitForServer();
        
       if(msg->parameters[2].value == "wp1"){
            goal.x = 2.2;
            goal.y = 0.0;
            goal.theta = 0.0;
        }
        else if (msg->parameters[2].value == "wp2"){
            goal.x = 0.0;
            goal.y = 2.2;
            goal.theta = M_PI/2;
        }
        else if (msg->parameters[2].value == "wp3"){
            goal.x = - 2.2;
            goal.y = 0.0;
            goal.theta = M_PI;
        }
        else if (msg->parameters[2].value == "wp4"){
            goal.x = 0.0;
            goal.y = - 2.2;
            goal.theta = - M_PI/2;
        }
        
        ac.sendGoal(goal);
        ac.waitForResult();
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}

    int main(int argc, char **argv) {
        ros::init(argc, argv, "leave_home_action", ros::init_options::AnonymousName);
        ros::NodeHandle nh("~");
        KCL_rosplan::LeaveHomeInterface leave_home(nh);
        leave_home.runActionInterface();
        return 0;
    }
