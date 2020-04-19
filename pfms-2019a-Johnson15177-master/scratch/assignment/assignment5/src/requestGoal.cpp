#include "ros/ros.h"
#include "std_msgs/String.h"
#include "a5_setup/RequestGoal.h"

bool requestGoal(a5_setup::RequestGoal::Request &req, a5_setup::RequestGoal::Response &res)
{
    double goal_point_x = req.x;
    double goal_point_y = req.y;
    res.ack = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "request_goal");
    ros::NodeHandle n;
    
    ros::ServiceServer service_ = n.advertiseService("request_goal", requestGoal);
    
    ros::spin();
    
    return 0;
}
