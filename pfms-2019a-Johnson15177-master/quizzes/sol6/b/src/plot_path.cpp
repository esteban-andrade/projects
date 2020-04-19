
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

//ROS Message Types
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"

//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//ROS Service
#include "a5_setup/RequestGoal.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <atomic>

namespace enc = sensor_msgs::image_encodings;



/**
 * This node attempts to draw the received path on an image
 */


class PfmsSample{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    image_transport::Subscriber sub3_;
    cv_bridge::CvImagePtr cvPtr_;
    ros::ServiceClient client_;

    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels
    double mapSize_;//! size of OgMap in pixels

    //! Is theer a better way to store data, instead of a structure?
    //! refer to Tutorial 7 exercise 3
    struct PoseArrayBuffer
    {
        std::mutex mtx;                 //! mutex to lock data
        std::atomic<bool> received;     //! bool to indicate data to be sent
        geometry_msgs::PoseArray poses; //! series of poses as PoseArray
        geometry_msgs::Pose vehPose;    //! the vehicle pose
        std::atomic<double> mapSize;    //! mapSize
        std::atomic<double> resolution; //! resolution
    };

    PoseArrayBuffer pathArrayBuffer_;//! Path received via callback


    struct ImageDataBuffer
    {
        //! Given these elements come in two's (image and time)
        //! Is there a better type of STL container rather than two seperate deques?
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex mtx;
    };

    ImageDataBuffer imageDataBuffer_;//! Container for image data


public:
    PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
    {
        sub1_ = nh_.subscribe("odom", 1000, &PfmsSample::odomCallback,this);
        image_transport::ImageTransport it(nh);
        sub3_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback,this);

        sub2_ = nh_.subscribe("path", 10, &PfmsSample::pathCallback,this);

        //Below is how to get parameters from command line, on command line they need to be _param:=value
        ros::NodeHandle pn("~");
        pn.param<double>("map_size", mapSize_, 20.0);
        pn.param<double>("resolution", resolution_, 0.1);

        //! @todo - Q2 : - create a client for the a5_help::RequestGoal on the "check_goal"
        //! Use member variable client_
        client_ = nh_.serviceClient<a5_setup::RequestGoal>("check_goal");

        // We store information allowing to convert from global to local
        pathArrayBuffer_.received=false;
        pathArrayBuffer_.mapSize=mapSize_;
        pathArrayBuffer_.resolution=resolution_;

    }

    ~PfmsSample()
    {
        cv::destroyWindow("view");
    }


    void pathCallback(const geometry_msgs::PoseArrayConstPtr & msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        ROS_INFO("New path received");
        ROS_INFO_STREAM("Received path has:" << msg->poses.size() << "elements");

        //! @todo - Q3 : Add the series of poses received from msg onto the pathArrayBuffer_.poses
        //! What do we need to do to access the pathArrayBuffer_
        pathArrayBuffer_.mtx.lock();
        pathArrayBuffer_.poses.poses.clear();
        for (unsigned int i=0;i<msg->poses.size();i++){
            pathArrayBuffer_.poses.poses.push_back(msg->poses.at(i));
        }
        pathArrayBuffer_.received = true;
        pathArrayBuffer_.mtx.unlock();

    }


    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        geometry_msgs::Pose pose=msg->pose.pose;
        pathArrayBuffer_.mtx.lock();
        pathArrayBuffer_.vehPose= pose;
        pathArrayBuffer_.mtx.unlock();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // Below code pushes the image and time to a deque, to share across threads
        try
        {
          if (enc::isColor(msg->encoding))
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
          else
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        imageDataBuffer_.mtx.lock();
        imageDataBuffer_.imageDeq.push_back(cvPtr_->image);

        // Additional Question: What does code below do?
        if((cvPtr_->image.cols !=200) || (cvPtr_->image.rows !=200)){
                ROS_WARN_THROTTLE(60, "The image is not 200 x 200 what has gone wrong!");
        }

        imageDataBuffer_.timeStampDeq.push_back(msg->header.stamp);

        if(imageDataBuffer_.imageDeq.size()>2){
            imageDataBuffer_.imageDeq.pop_front();
            imageDataBuffer_.timeStampDeq.pop_front();
        }

        imageDataBuffer_.mtx.unlock();

    }


    void seperateThread() {
       /**
        * The below loop runs until ros is shutdown, to ensure this thread does not remain
        * a zombie thread
        *
        * The loop locks the buffer, checks the size
        * And then pulls items: the pose and timer_t
        * You can think if these could have been combined into one ...
        */

        /// The below gets the current Ros Time
        ros::Time timeImage = ros::Time::now();
        cv::Mat image;
        count_ =0;
        cv::namedWindow("path",CV_WINDOW_NORMAL);
        cv::startWindowThread();
        cv::waitKey(50);

        while (ros::ok()) {

            bool imageOK=false;

            //! Lock image buffer, take one message from deque and unlock it
            imageDataBuffer_.mtx.lock();
            if(imageDataBuffer_.imageDeq.size()>0){
                image = imageDataBuffer_.imageDeq.front();
                timeImage = imageDataBuffer_.timeStampDeq.front();
                imageDataBuffer_.imageDeq.pop_front();
                imageDataBuffer_.timeStampDeq.pop_front();
                imageOK=true;
            }
            imageDataBuffer_.mtx.unlock();

            // Update GUI Window on new OgMap received
            if(imageOK){

                if(pathArrayBuffer_.received){

                  ROS_INFO("!!! Got new path");
                  cv::Mat rgbImage;
                  cv::cvtColor(image,rgbImage,CV_GRAY2RGB);

                  pathArrayBuffer_.mtx.lock();

                  //! @todo - Q4
                  //!
                  //! Convert the path to pixel coordinates and draw on OgMap
                  //! This will be reverse of the code supplied in gen_path CallBackFunc

                  for(unsigned int i=0;i<pathArrayBuffer_.poses.poses.size();i++){

                    geometry_msgs::Pose pose = pathArrayBuffer_.poses.poses.at(i);
                    ROS_INFO_STREAM("*** path elem:" << i);
                    ROS_INFO_STREAM("global position [x,y]=[" << pose.position.x << ","  << pose.position.y << "]");
                    double offset = (pathArrayBuffer_.mapSize/pathArrayBuffer_.resolution)/2;

                    double xpos = pose.position.x - pathArrayBuffer_.vehPose.position.x;
                    double ypos = pose.position.y - pathArrayBuffer_.vehPose.position.y;
                    ROS_INFO_STREAM("local  position [x,y]=[" << xpos << ", " << ypos << "]");

                    double xD = (xpos/ pathArrayBuffer_.resolution)+offset;
                    double yD = (-ypos/ pathArrayBuffer_.resolution)+offset;

                    ROS_INFO_STREAM("path elem:" << i << "position [" << xD << ", " << yD << "]");

                    cv::circle(rgbImage, cv::Point((xD), (yD)), 3, CV_RGB(255,0,0),-1);

                    //! INFO
                    //!  Use the circle to draw the pose
                    //! cv::circle(rgbImage, cv::Point((xD), (yD)), 3, CV_RGB(255,0,0),-1);
                  }


                   //! @todo - Q5 : call the client for a5_help::RequestGoal with the goal the last (final) pose of the path

                   geometry_msgs::Pose pose = pathArrayBuffer_.poses.poses.back();

                   pathArrayBuffer_.poses.poses.clear();
                   pathArrayBuffer_.received=false;
                   pathArrayBuffer_.mtx.unlock();

                   a5_setup::RequestGoal srv;
                   srv.request.x = pose.position.x;
                   srv.request.y = pose.position.y;

                   if (client_.call(srv)){
                     ROS_INFO("Responce: %d", srv.response.ack);
                   }
                   else{
                     ROS_ERROR("Failed to call service request_goal");
                   }

                   cv::putText(rgbImage, "Received Path.", cv::Point(30,30),
                       cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(200,200,250), 1, CV_AA);

                   cv::imshow("path", rgbImage);


                }
                else{
                    //std::cout << "No new path" << std::endl;
                }

                cv::waitKey(5);
            }
            // This delay slows the loop down for the sake of readability
            std::this_thread::sleep_for (std::chrono::milliseconds(50));
        }
        std::cout << __func__ << " thread terminated" << std::endl;
    }

};


int main(int argc, char **argv)
{


  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "draw_path");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  std::cout << "STARTING" << std::endl;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on teh function desired
   */
  std::shared_ptr<PfmsSample> gc(new PfmsSample(nh));
  std::thread t(&PfmsSample::seperateThread,gc);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  t.join();

  return 0;
}

