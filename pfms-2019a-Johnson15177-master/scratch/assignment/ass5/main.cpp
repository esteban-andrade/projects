#include "ros/ros.h"
#include "temp.h"

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "assignment5");
    ros::NodeHandle nh;
    
    std::shared_ptr<PfmsSample> gc(new PfmsSample(nh));
    std::thread t(&PfmsSample::separateThread, gc);
    
    ros::spin();
    
    ros::shutdown();
    
    t.join();
    
    return 0;
}
