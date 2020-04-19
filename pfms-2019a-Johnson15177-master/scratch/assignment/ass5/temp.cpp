#include "temp.h"

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

}

void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
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
        rate_limiter.sleep();
    }
}
