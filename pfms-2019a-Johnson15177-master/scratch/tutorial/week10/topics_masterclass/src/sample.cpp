
#include "sample.h"


/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Subscribing to images (which require a image transport interpreter)
 * - Publishing images
 * - The need to exchange data betweena seperate thread of execution and callbacks
 */


PfmsSample::PfmsSample(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
{
    //Subscribing to odometry
    sub1_ = nh_.subscribe("odom", 1000, &PfmsSample::odomCallback,this);
    //Subscribing to laser
    sub2_ = nh_.subscribe("base_scan_1", 10, &PfmsSample::laserCallback,this);

    //Subscribing to image
    // unlike other topics this requires a image transport rather than simply a node handle
    image_transport::ImageTransport it(nh);
    sub3_ = it.subscribe("map_image/full", 1, &PfmsSample::imageCallback,this);

    //Publishing an image ... just to show how
    image_pub_ = it_.advertise("test/image", 1);


    //Below is how to get parameters from command line, on command line they need to be _param:=value
    //For example _map_sise:=20.0
    ros::NodeHandle pn("~");
    pn.param<double>("resolution", resolution_, 0.1);

    count_ =0;
}

PfmsSample::~PfmsSample()
{
//    cv::destroyWindow("view");
}



// A callback for odometry
void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    /**
     * @todo - Ex 1 : Obtain a pose (x,y yaw) from nav_msgs/Odometry
     *
     * - On command line type 'rosmsg show nav_msgs/Odometry'
     * - The position and orientation are in two seperate parts of the message
     * - The orinetation is provided as a quaternion
     * - Which angle to we need?
     * - Ros has a 'tf' library with a helper function to get yaw from the quaternion
     * - http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     * - Consider: Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
     * - Where is time of this message stored
     */

    /*ROS_INFO_STREAM("x: " << msg->pose.pose.position.x << ", y: "
                    << msg->pose.pose.position.y << ", yaw: "
                    << tf::getYaw(msg->pose.pose.orientation));*/

    /*std::cout << "x: " << msg->pose.pose.position.x << ", y: "
              << msg->pose.pose.position.y << ", yaw: "
              << tf::getYaw(msg->pose.pose.orientation) << std::endl;*/

   /* std::cout << "(" << msg->pose.pose.position.x << " ," << msg->pose.pose.position.y << ")" << std::endl;
    std::cout << "The orientation is " << msg->pose.pose.orientation << std::endl;*/
    //ROS_INFO_SYSTEM("yaw: " << tf::getYaw(msg->pose.pose.orientation));

    pose_mtx_.lock();
    pose_=msg->pose.pose;
    pose_mtx_.unlock();
}



void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  /**
   * @todo - Ex 2 : Find the closest point [x,y] to the robot using sensor_msgs::LaserScan
   *
   * - On command line type 'rosmsg show sensor_msgs/LaserScan'
   * - What are we provided in this message?
   * - Do we have the inforamtion in this message to find the closest point?
   * - What part of the message do we need to iterate over?
   * - How do we convert from range data to [x,y] (this is known as polar to cartesian)
   * - https://www.mathsisfun.com/polar-cartesian-coordinates.html
   * - Where is time of this message stored?
   * - Is the closest point identified the same as the one you see as closest on the stage simulator?
   * - Why is this the case?
   */

    double closest_range = msg->range_max;
    int closest_index = -1;
    for(int i=0; i<msg->ranges.size(); i++)
    {
        if( msg->ranges.at(i) < closest_range)
        {
            closest_range = msg->ranges.at(i);
            closest_index = i;
        }
    }

    double closest_angle = msg->angle_min + closest_index * msg->angle_increment;
    double x = closest_range * cos(closest_angle);
    double y = x / cos(closest_angle);
    //double y = closest_range * sin(closest_angle);

    std::cout << "Closest thing is at " << x << ", " << y << std::endl;
    //std::cout << "Range is " << closest_range << std::endl;


    point_mtx_.lock();
    closestPoint_.x = x;
    closestPoint_.y = y;
    point_mtx_.unlock();

}

void PfmsSample::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //! Below code converts from sensor message to a pointer to an opencv image and time to a deque, to share across threads
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

    //To obtain image we use below
    img_mtx_.lock();
    image_ = cvPtr_->image;

    /**
     * @todo Ex 3 : Find the closest point [x,y] to the robot using the cv::Mat image
     *
     * - Where is time of this message stored?
     * - What information do we convert from the OgMap pixels to [x,y]
     * - Where is 0,0 on the image?
     *  o------> j
     *  |
     *  |
     *  v i
     * - Where is (0,0) on the OgMap?
     * - Is the closest point identified the same as the one you see as closest on the stage simulator?
     * - Why is this the case?
     */

    // An example of a loop examining row and col values
    //    for (int i = 0; i<image.rows; i++) {
    //        for (int j = 0; j <image.cols; j++) {
    //            unsigned char &pixel = image.at<unsigned char>(i, j);
    //            if (pixel > thresh) {
    //
    //            }
    //        }
    //    }
    
    img_mtx_.unlock();

}


void PfmsSample::seperateThread() {
   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    */

    cv_bridge::CvImage cv_image; // this cv_bridge object is used for publishing images in ROS

    /// The below gets the current Ros Time
    ros::Time timeSample = ros::Time::now();

    //! What does this rate limiter do?
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) {

      /**
       * @todo Ex 4 : Find the closest point [x,y] to the robot using the robot pose and laser data
       *
       * - If we need to combine the information and use it in this seperate thread what do we need to consider
       * - Is the closest point identified the same as the one you see as closest on the stage simulator?
       * - Why is this the case?
       */
    //In the below we use a special mutex that allows trying to lock for a certain amount of time and then giving up
    //This allows us to avoid deadlock, and still keep the thraed running within some guarantees of max run-time
    bool have_pose = false;
    bool have_point = false;

    geometry_msgs::Pose pose;
    geometry_msgs::Point pt;
    if(pose_mtx_.try_lock_for(std::chrono::milliseconds(5))) //This tries to mutex lock for 5ms and then gives up
    {
        pose = pose_;
        pose_mtx_.unlock();
        have_pose = true;
    }
    
    if (point_mtx_.try_lock_for(std::chrono::milliseconds(5))) //This tries to mutex lock for 5ms and then gives up
    {
            pt = closestPoint_;
            point_mtx_.unlock();
            have_point =true;
    }
    
    if(have_pose && have_point) // if we have both nearest point and pose we can compute it in global
    {
        double yaw = tf::getYaw(pose.orientation);
        // we need to transform closest point to global reference frame
        // easiest is to be in polar coordinates
        double d = pow(pow(pt.x, 2) + pow(pt.y, 2), 0.5); // this is how far the point is in local frame 
        double theta = atan2(pt.y, pt.x); // this is the angle to the point in local frame
        theta += yaw; // the robot has a yaw, so let's adjust for this
        pt.x = d*cos(theta); // this is new computed point x with angle adjusted
        pt.y = d*sin(theta); // this is new computed point y with angle adjusted
        
        // we need to shift the point by th robot location to get into global coordinates
        pt.x += pose.position.x; // now the point is in global coordinates
        pt.y += pose.position.y; // now the point is in global coordinates
        std::cout << theta << std::endl;
        std::cout << "Global closest obstacle: (x, y)=(" << pt.x << ", " << pt.y << ")" << std::endl;
    }
    
    //For the image we can lock mutex and then check if image is empty
    //by doing so, we will not get into a position whereby this thread get's to an empty image first (no OgMap)
    //recieved at this stage, and we could segfault accessing memory that is not available as yet
    img_mtx_.lock();
    if(!image_.empty())
    {
    // EX3-------------------------------------------
        // loop examining row and col values
        double closest_distance = pow(image_.rows, 2) + pow(image_.cols, 2); //definately will be less than this
        cv::Point closest_point(0, 0);
        
        for(int i = 0; i < image_.rows; i++)
        {
            for(int j = 0; j < image_.cols; j++)
            {
                unsigned char &pixel = image_.at<unsigned char>(i, j);
                if(pixel == 0)
                {
                    double d = pow(pow((i-(image_.rows/2)), 2) + pow((j-(image_.cols/2)), 2), 0.5);
                    if(d<closest_distance)
                    {
                        closest_distance = d;
                        closest_point.x = j;
                        closest_point.y = i;
                    }
                }
            }
        }
        closest_distance*= resolution_;
        //ex 5
        //below takes the cv::Mat image which is single channel (greyscale) and converts it into an RGB image
        cv::cvtColor(image_, cv_image.image, CV_GRAY2RGB);
        img_mtx_.unlock(); //we can unlock here habing finished with image_
        
        std::cout << "Closest occupied point is: " << closest_distance << " m away" << std::endl;
        
        //draw a circle at closest point of size 3 pixels and in green colour
        cv::circle(cv_image.image, closest_point, 3, CV_RGB(0, 255, 0), 1);
        
        //to publish - send the image we need to specify the encoding and make a header
        cv_image.encoding = "bgr8";
        cv_image.header = std_msgs::Header();
        
        //we now publish the image and use the inbuilt ros function to convert cv_image to an image message
        image_pub_.publish(cv_image.toImageMsg());
        
    }
    else {
        img_mtx_.unlock();
    }
    
      /**
       * @todo Ex 5 : Mark the closest point on a RGB version of the OgMap image
       *
       * - If we need to combine the information and use it in this seperate thread what do we need to consider
       * - Is the closest point identified the same as the one you see as closest on the stage simulator?
       * - Why is this the case?
       */


      /*
       * The section is commented out initialy and can be used in Ex 5
        //If the image is not empty (which means it has data)
        if(!image.empty()){
          //Below code takes the cv::Mat image which is single channel (grayscale) and converts it into a RGB image
          cv_bridge::CvImage cv_image;
          cv::cvtColor(image,cv_image.image, CV_GRAY2RGB);
          // create a cv::Point at the centre of the image
          cv::Point pt1(image.cols/2,image.rows/2);
          // draw a circle using the point of size 3 pixels and in green color
          cv::circle(cv_image.image, pt1, 3, CV_RGB(0,255, 0) , 1);

          // To publish - send the image we need to specify the encoding and make a header
          cv_image.encoding = "bgr8";
          cv_image.header = std_msgs::Header();
          // We now publish the image and use the inbuilt ros function to convert cv_image to a image message
          image_pub_.publish(cv_image.toImageMsg());
        }
      */
        rate_limiter.sleep();
    }


}

