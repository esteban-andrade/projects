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

//! All the messages we need are here
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
#include "processing.h"
#include <atomic>

//#include "image_processing.h"


namespace enc = sensor_msgs::image_encodings;

/**
 * This node shows some connections and publishing images
 */


class FrontierExplorer{

public:
  /*! @brief FrontierExplorer constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    FrontierExplorer(ros::NodeHandle nh);

  /*! @brief FrontierExplorer destructor.
   *
   *  Will tear down the object
   */
    ~FrontierExplorer();
    
  /*! @brief Request Goal service callback
   *
   *  @param req The requested goal.
   *  @param res The responce
   *
   *  @return bool - Will return true to indicate the request has sucsedded
   */
    /*bool requestGoal(a5_setup::RequestGoal::Request &req,
        a5_setup::RequestGoal::Response &res);*/

  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

 /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

 /*! @brief ImageConst Callback
   *
   *  @param sensor_msgs::ImageConstPtr - The imageconst message
   *  @note This function and the declaration are ROS specific
   */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */
   //imageProcessing_
    void seperateThread(); 

    double pixeltoGlobal(double pixelx, double pixely);

    void transformCoordinates();
    
    void findCells();
    
    void findGoalPoint();
    void findGoalAngle();
    
    
public:
    ros::NodeHandle nh_;

private:
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_; //! Image publisher

    ros::Subscriber sub1_;
    image_transport::Subscriber sub2_;
    ros::Subscriber sub3_;

    ros::ServiceClient client_;

    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels
    
    geometry_msgs::Point goalPoint_;
    
    std::mutex point_mtx_;
    std::atomic<bool> goalRequested_;
    
    cv_bridge::CvImagePtr cvPtr_;
    cv_bridge::CvImage cv_image;
    
    cv::Mat image_; //! The OgMap as an image
    std::mutex img_mtx_; //! Mutex to control access to image
    
    double theta;
    double angle;
    double laser_angle;
    double laser_index;
    double max_laser_range;
    double angle_increment;
    double angle_min;
    double angle_max;
    
    struct RobotPose
    {
        double x;
        double y;
        std::mutex mtx;
    }; RobotPose robotPose_;
    std::vector<RobotPose> robotPosition;
    
    
    struct Frontier
    {
        int x;
        int y;
    }; Frontier frontier_;
    std::vector<Frontier> frontierpt;
    
    struct Unknown
    {
        int x;
        int y;
    }; Unknown unknown_;
    std::vector<Unknown> unknownpt;
    
    struct Goal
    {
        int x;
        int y;
        double distance;
        double angle;
    }; Goal goal_;
    std::vector<Goal> goalpt;
    
    double goal_Pointx;
    double goal_Pointy;
    
    struct Goal_Global
    {
        double x;
        double y;
    }; Goal_Global goal_global_;
    
    double goal_Globalx;
    double goal_Globaly;
    
    //FrontierExplorer frontier_explorer; //! Object of class

    
};
