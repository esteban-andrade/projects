
#include "sample.h"


/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Subscribing to images (which require a image transport interpreter)
 * - Publishing images
 *
 * The code is not intended to provide a
 */


PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh), imageProcessing_()
{
  //Subscribing to odometry
  sub1_ = nh_.subscribe("odom", 1000, &PfmsSample::odomCallback,this);

  //Subscribing to image
  // unlike other topics this requires a image transport rather than simply a node handle
  image_transport::ImageTransport it(nh);
  sub2_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback,this);

  //Publish a velocity ... to control the robot
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

  //Allowing an incoming service on /face_goal, this invokes the faceGoal function
  service_ = nh_.advertiseService("face_goal", &PfmsSample::faceGoal,this);

  //Below is how to get parameters from command line, on command line they need to be _param:=value
  //For example _resolution:=0.1
  ros::NodeHandle pn("~");
  pn.param<double>("resolution", resolution_, 0.1);

}

PfmsSample::~PfmsSample()
{
//    cv::destroyWindow("view");
}


bool PfmsSample::faceGoal(a5_setup::RequestGoal::Request  &req,
             a5_setup::RequestGoal::Response &res)
{
  //When an incoming call arrives, we can respond to it here
  ROS_INFO_STREAM("request: [x,y]=[" << req.x << "," << req.y);

  //! @todo Ex04 : Adjust the code so it returns a `true` in the acknowledgment of the service call if the point
  //! can be reached in a straight line from current robot pose, only traversing free space.

  //BEGIN
  // Below code will be useful
  //  //! Lock image buffer, take one message from deque and unlock it
  //  cv::Mat image;
  //  imageDataBuffer_.mtx.lock();
  //  if(imageDataBuffer_.imageDeq.size()>0){
  //      image = imageDataBuffer_.imageDeq.front();
  //      timeImage = imageDataBuffer_.timeStampDeq.front();
  //      imageDataBuffer_.imageDeq.pop_front();
  //      imageDataBuffer_.timeStampDeq.pop_front();
  //  }
  //  imageDataBuffer_.mtx.unlock();
  //END

  res.ack = true;

  ROS_INFO_STREAM("sending back response:" << res.ack);
  return true;

}

void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    //Let's get the pose out from odometry message
    //rosmsg show nav_msgs/Odometry
    geometry_msgs::Pose pose=msg->pose.pose;

    poseDataBuffer_.mtx.lock();
    poseDataBuffer_.poseDeq.push_back(pose);
    poseDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
    if(poseDataBuffer_.poseDeq.size()>2){
        poseDataBuffer_.poseDeq.pop_front();
        poseDataBuffer_.timeStampDeq.pop_front();
    }
    poseDataBuffer_.mtx.unlock();
}


void PfmsSample::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //! Below code pushes the image and time to a deque, to share across threads
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
    imageDataBuffer_.timeStampDeq.push_back(msg->header.stamp);
    if(imageDataBuffer_.imageDeq.size()>2){
        imageDataBuffer_.imageDeq.pop_front();
        imageDataBuffer_.timeStampDeq.pop_front();
    }
    imageDataBuffer_.mtx.unlock();
}


void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    //! What does this rate limiter do?
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) {

      //! @todo Ex05 : Control the robot so it turns to face the point requested (and does not move forward).
      //! Yor node should also allow for any new requests to overwrite the current actioned goal.
      //! What data do we need?
        geometry_msgs::Twist cmdvel;
        cmdvel.linear.x = 0.1; //Linear Velocity - V
        cmdvel.angular.z = 0;  //Angular Velocity - Omage
        ROS_INFO_STREAM("sending...[" <<  cmdvel.linear.x << "," <<  cmdvel.angular.z << "]");
        cmd_vel_pub_.publish(cmdvel);

        rate_limiter.sleep();

    }
}

