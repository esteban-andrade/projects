#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

#include "ros/ros.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//! All the services .. RequestGoal is part of a5_setup, not a standard service
#include "a5_setup/RequestGoal.h"
#include <atomic>
#include "image_processing.h"

namespace enc = sensor_msgs::image_encodings;

class PfmsSample{
    public:
    
        PfmsSample(ros::NodeHandle nh);
        ~PfmsSample();
        bool faceGoal(a5_setup::RequestGoal::Request &req, a5_setup::RequestGoal::Response &res);
        
        void odomCallback(const nav_msgs::OdometryConstPtr& msg);
        
        void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);
        
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        
        void separateThread();
        
    public:
        ros::NodeHandle nh_;
        
    private:
        image_transport::ImageTransport it_;
        image_transport::Publisher image_pub_;
        ros::Publisher cmd_vel_pub_;
        
        ros::Subscriber sub1_;
        ros::Subscriber sub2_;
        image_transport::Subscriber sub3_;
        cv_bridge::CvImagePtr cvPtr_;
        ros::ServiceServer service_;

        int count_;//! A counter to allow executing items on N iterations
        double resolution_;//! size of OgMap in pixels

        cv::Mat image_; //! The OgMap as an image
        std::mutex img_mtx_; //! Mutex to control access to image

        geometry_msgs::Pose pose_; //! Pose of the robot
        std::timed_mutex pose_mtx_; //! Mutex to control access to pose
    
        geometry_msgs::Point closestPoint_; //! Nearest point as determined by laser
        std::timed_mutex point_mtx_; //! Mutex to control access to nearest point
};
