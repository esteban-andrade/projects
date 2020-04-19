
#include "frontier.h"

using namespace cv;

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Subscribing to images (which require a image transport interpreter)
 * - Publishing images
 *
 * The code is not intended to provide a
 */


FrontierExplorer::FrontierExplorer(ros::NodeHandle nh)
    : nh_(nh), it_(nh)//, frontier_explorer()
{
    //Subscribing to odometry
    sub1_ = nh_.subscribe("odom", 1000, &FrontierExplorer::odomCallback,this);

    //Subscribing to image
    // unlike other topics this requires a image transport rather than simply a node handle
    image_transport::ImageTransport it(nh);
    sub2_ = it.subscribe("map_image/full", 1, &FrontierExplorer::imageCallback,this);
  
    //Subscribing to laser
    sub3_ = nh_.subscribe("base_scan_1", 10, &FrontierExplorer::laserCallback,this);

    //Publishing an image over the ROS framework
    image_pub_ = it_.advertise("map_image/fbe", 1);

    client_ = nh_.serviceClient<a5_setup::RequestGoal> ("request_goal");
    

  //Below is how to get parameters from command line, on command line they need to be _param:=value
  ros::NodeHandle pn("~");
  pn.param<double>("resolution", resolution_, 0.1);

  count_ = 0;
}

FrontierExplorer::~FrontierExplorer()
{

}

void FrontierExplorer::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    //Let's get the pose out from odometry message
    //rosmsg show nav_msgs/Odometry
    geometry_msgs::Pose pose = msg->pose.pose;
    
    robotPose_.mtx.lock();
    robotPose_.x = pose.position.x;
    robotPose_.y = pose.position.y;
    robotPose_.mtx.unlock();
}


void FrontierExplorer::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    img_mtx_.lock();
    image_ = cvPtr_->image;
    img_mtx_.unlock();
    
}

void FrontierExplorer::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    point_mtx_.lock();
    laser_index = msg->ranges.size();
    max_laser_range = msg->range_max;
    angle_increment = msg->angle_increment;
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    point_mtx_.unlock();
    
}

/*double FrontierExplorer::pixeltoGlobal(double pixelx, double pixely)
{
    Globalx = ((pixely - (image_.cols/2)) * resolution_) + robotPose_.x;
    goal_Globaly = -((pixelx - (image_.rows/2)) * resolution_) + robotPose_.y;
        
    
    return (goal_Globalx, goal_Globaly);
}*/

void FrontierExplorer::transformCoordinates()
{
    std::cout << "-------------------------------------" << std::endl;
    Processing processing;
    //pixel to global
    goal_Globalx = processing.pixeltoGlobalx(image_, goal_Pointy, robotPose_.x, resolution_);
    
    goal_Globaly = processing.pixeltoGlobaly(image_, goal_Pointx, robotPose_.y, resolution_);
    
    //goal_Globalx = ((goal_Pointy - (image_.cols/2)) * resolution_) + robotPose_.x;
    //goal_Globaly = -((goal_Pointx - (image_.rows/2)) * resolution_) + robotPose_.y;
    
    std::cout << "goal pixel " << goal_Pointx << ", " << goal_Pointy << std::endl;
    
    std::cout << "pixel to global  " << goal_Globalx << ", " << goal_Globaly << std::endl;
    
    a5_setup::RequestGoal srv;
    srv.request.x = goal_Globalx;
    srv.request.y = goal_Globaly;
    if(client_.call(srv))
    {
        std::cout << "Worked" << std::endl;
        
        
        /*cv_bridge::CvImage cv_image;
        cv::cvtColor(image_,cv_image.image, CV_GRAY2RGB);
        
        cv_image.encoding = "bgr8";
        cv_image.header = std_msgs::Header();
        image_pub_.publish(cv_image.toImageMsg());*/
    }
    
    //global to pixel
    double offset = (image_.cols/2);
    double pixel_Coor_x;
    double pixel_Coor_y;
    pixel_Coor_x = processing.globaltoPixelx(image_, goal_Globaly, robotPose_.y, resolution_, offset);
    pixel_Coor_y = processing.globaltoPixely(image_, goal_Globalx, robotPose_.x, resolution_, offset);
    
    
    
    //pixel_Coor_x = abs(((goal_Globaly - robotPose_.y)/resolution_) - offset);
    //pixel_Coor_y = ((goal_Globalx - robotPose_.x)/resolution_) + offset;
    
    //FrontierExplorer frontierexplorer_;
    
    std::cout << "global to pixel " << pixel_Coor_x << ", " << pixel_Coor_y << std::endl;
    //std::cout << "robot global " << robotPose_.x << ", " << robotPose_.y << std::endl;
    //std::cout << "Goal pixel " << goal_Pointx << ", " << goal_Pointy << std::endl;
    //std::cout << "Goal global " << goal_Globalx << ", " << goal_Globaly << std::endl;
    
}

void FrontierExplorer::findGoalPoint()
{
    double closest_goal = goalpt.at(0).distance;
    for(int z = 0; z<goalpt.size(); z++)
    {
        if(closest_goal > goalpt.at(z).distance)
        {
                closest_goal = goalpt.at(z).distance;
                goal_Pointx = goalpt.at(z).x;
                goal_Pointy = goalpt.at(z).y;
        }
    }
}

void FrontierExplorer::findGoalAngle()
{
    double unknownx, unknowny;
    double distance_unknown_goal = sqrt(pow((unknownpt.at(0).x-goal_Pointx),2) + 
    pow((unknownpt.at(0).y-goal_Pointy),2));
        
    for(int i = 0; i < unknownpt.size(); i++)
    {
            
        double check_distance = sqrt(pow((unknownpt.at(i).x-goal_Pointx),2) + 
        pow((unknownpt.at(i).y-goal_Pointy),2));
        if(distance_unknown_goal > check_distance)
        {
            distance_unknown_goal = check_distance;
            unknownx = unknownpt.at(i).x;
            unknowny = unknownpt.at(i).y;
        }
    }
        
    double dx = unknownx - goal_Pointx;
    theta = acos(abs(dx)/distance_unknown_goal);
    double laserx, lasery;
    //right
    if(unknownx == goal_Pointx && unknowny > goal_Pointy)
    {
        angle = 0;
    }
    // top right
    if(unknownx < goal_Pointx && unknowny > goal_Pointy)
    {
        angle = theta;
    }
    //top
    if(unknownx < goal_Pointx && unknowny == goal_Pointy)
    {
        angle = 1.5;
    }
    //top left
    if(unknownx < goal_Pointx && unknowny < goal_Pointy)
    {
        angle = 3.14 - theta;
    }
    //left
    if(unknownx == goal_Pointx && unknowny < goal_Pointy)
    {
        angle = 3.14;
    }
    //bottom left
    if(unknownx > goal_Pointx && unknowny < goal_Pointy)
    {
        angle = 3.14 + theta;
    }
    //bottom
    if(unknownx > goal_Pointx && unknowny == goal_Pointy)
    {
        angle = 4.7;
    }
    //bottom right
    if(unknownx > goal_Pointx && unknowny > goal_Pointy)
    {
        angle = 6.28 - theta;
    }    
}

void FrontierExplorer::findCells()
{
    cv_bridge::CvImage cv_image;
    cv::cvtColor(image_,cv_image.image, CV_GRAY2RGB);

             
    Vec3b free(255,255,255),
        occupied(0,0,0),
        unknown(127,127,127),
        frontier(255,0,0),
        goal(0,255,0),
        frontierCell(0,0,255),
        robot(100,200,100);
        
        /* colours the cell blue  */
        for(int i = 0; i < image_.rows; i++)
        {
            for(int j = 0; j < image_.cols; j++)
            {
                Vec3b &pixel = cv_image.image.at<Vec3b>(i,j);
                if(pixel == free)
                {
                    for(int x = 0; x < 3; x++)
                    {
                        for(int y = 0; y < 3; y++)
                        {
                            Vec3b &pixel2 = cv_image.image.at<Vec3b>((i+x-1), (j+y-1));
                            if(pixel2 == unknown)
                            {
                                pixel = frontier;
                                //pixel2 = frontierCell;
                                unknown_.x = i+x-1;
                                unknown_.y = j+y-1;
                                unknownpt.push_back(unknown_);
                                x = 3;
                                y = 3;
                            }
                        }
                    }
                }
            }
        }
        
        /* gets all goal positions and stores it into goalpt*/
        for(int i = 0; i < image_.rows; i++)
        {
            for(int j = 0; j < image_.cols; j++)
            {
                Vec3b &pixel = cv_image.image.at<Vec3b>(i,j);
                if(pixel == free)
                {
                    for(int x = 0; x < 3; x++)
                    {
                        for(int y = 0; y < 3; y++)
                        {
                            Vec3b &pixel2 = cv_image.image.at<Vec3b>((i+x-1), (j+y-1));
                            if(pixel2 == frontier)
                            {
                                goal_.x = i;
                                goal_.y = j;
                                goal_.distance = sqrt(pow((goal_.x-image_.rows/2),2) + 
                                    pow((goal_.y-image_.cols/2),2));
                                goalpt.push_back(goal_);
                                x = 3;
                                y = 3;
                            }
                        }
                    }
                }
            }
        }        
        
        findGoalPoint();
        /* colours the closest goal point */
        Vec3b &pixel3 = cv_image.image.at<Vec3b>(goal_Pointx, goal_Pointy);
        pixel3 = goal;
        
        findGoalAngle();
        //std::cout << angle << std::endl;        
        //std::cout << "goal in separate " << goal_Pointx << ", " << goal_Pointy << std::endl;
        
        cv::Point start;
        start.x = goal_Pointy;
        start.y = goal_Pointx;
        for(int i = 0; i < laser_index; i++)
        {
            double temp_angle = i*angle_increment + angle;
            double desx, desy;
            desx = ((max_laser_range * sin(temp_angle))/resolution_) + goal_Pointy;
            desy = ((max_laser_range * cos(temp_angle))/resolution_) + goal_Pointx;
            cv::Point end(desx, desy);
            cv::LineIterator lineIterator(cv_image.image, start, end);
            
            for(int j = 0; j < lineIterator.count; j++, lineIterator++)
            {
                Vec3b *pixel = (Vec3b*)*lineIterator;
                //*pixel = frontierCell;
                if(*pixel == frontier)
                {
                    *pixel = frontierCell;
                }
                if(*pixel == occupied)
                {
                    break;
                }
                if(*pixel == unknown)
                {
                    break;
                }
            }
        }
        
        for(int i = 0; i < image_.rows; i++)
        {
            for(int j = 0; j < image_.cols; j++)
            {
                Vec3b &pixel = cv_image.image.at<Vec3b>(i,j);
                if(pixel == frontierCell)
                {
                    frontier_.x = i;
                    frontier_.y = j;
                    frontierpt.push_back(frontier_);
                }
            }
        }
        //pixel3 = goal;
        
        Vec3b &pixel4 = cv_image.image.at<Vec3b>(100, 100);
        pixel4 = robot;
        
        cv_image.encoding = "bgr8";
        cv_image.header = std_msgs::Header();
        image_pub_.publish(cv_image.toImageMsg());
        
        goalpt.clear();
        unknownpt.clear();

}

void FrontierExplorer::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */
    double yaw = 0;
    double x = 0;
    double y = 0;    
        
    ros::Rate rate_limiter(1.0);
    while (ros::ok()) 
    {
        if(!image_.empty())
        {
            findCells();
            transformCoordinates();
        
        }
        rate_limiter.sleep();
    }
}

