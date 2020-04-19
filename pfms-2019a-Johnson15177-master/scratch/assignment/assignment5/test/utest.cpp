#include <gtest/gtest.h>
#include <climits>

#include <ros/package.h> //This tool allows to identify the path of the package on your system

#include <cv_bridge/cv_bridge.h>
//#include "../src/image_processing.h"
#include "../src/processing.h"

using namespace std;

TEST(TransformCoordinates, PixeltoGlobal_TR)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 80;
    int pixely = 120;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(2, Globalx);
    ASSERT_EQ(2, Globaly);
}

TEST(TransformCoordinates, PixeltoGlobal_T)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 80;
    int pixely = 100;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(0, Globalx);
    ASSERT_EQ(2, Globaly);
}

TEST(TransformCoordinates, PixeltoGlobal_TL)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 80;
    int pixely = 80;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(-2, Globalx);
    ASSERT_EQ(2, Globaly);
}

TEST(TransformCoordinates, PixeltoGlobal_L)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 100;
    int pixely = 80;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(-2, Globalx);
    ASSERT_EQ(0, Globaly);
}

TEST(TransformCoordinates, PixeltoGlobal_BL)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 120;
    int pixely = 80;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(-2, Globalx);
    ASSERT_EQ(-2, Globaly);
}

TEST(TransformCoordinates, PixeltoGlobal_B)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 120;
    int pixely = 100;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(0, Globalx);
    ASSERT_EQ(-2, Globaly);
}

TEST(TransformCoordinates, PixeltoGlobal_BR)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 120;
    int pixely = 120;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(2, Globalx);
    ASSERT_EQ(-2, Globaly);
}

TEST(TransformCoordinates, PixeltoGlobal_R)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    int pixelx = 100;
    int pixely = 120;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double Globalx = test.pixeltoGlobalx(image_, pixely, robotPosex, resolution_);
    double Globaly = test.pixeltoGlobaly(image_, pixelx, robotPosey, resolution_);
    
    ASSERT_EQ(2, Globalx);
    ASSERT_EQ(0, Globaly);
}

TEST(TransformCoordinates, GlobaltoPixel_TR)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = 5;
    double Globaly = 5;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    double offset = (image_.cols/2);
    int pixels = (int) mapSize / resolution_; 
    
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(50, pixelx);
    ASSERT_EQ(50, pixely);
}

TEST(TransformCoordinates, GlobaltoPixel_T)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = 0;
    double Globaly = 5;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    
    int pixels = (int) mapSize / resolution_; 
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    double offset = (image_.cols/2);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(50, pixelx);
    ASSERT_EQ(100, pixely);
}

TEST(TransformCoordinates, GlobaltoPixel_TL)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = -5;
    double Globaly = 5;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    
    
    int pixels = (int) mapSize / resolution_; 
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    double offset = (image_.cols/2);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(50, pixelx);
    ASSERT_EQ(50, pixely);
}

TEST(TransformCoordinates, GlobaltoPixel_L)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = -5;
    double Globaly = 0;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    
    
    int pixels = (int) mapSize / resolution_; 
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    double offset = (image_.cols/2);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(100, pixelx);
    ASSERT_EQ(50, pixely);
}

TEST(TransformCoordinates, GlobaltoPixel_BL)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = -5;
    double Globaly = -5;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    
    
    int pixels = (int) mapSize / resolution_; 
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    double offset = (image_.cols/2);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(150, pixelx);
    ASSERT_EQ(50, pixely);
}

TEST(TransformCoordinates, GlobaltoPixel_B)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = 0;
    double Globaly = -5;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    
    
    int pixels = (int) mapSize / resolution_; 
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    double offset = (image_.cols/2);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(150, pixelx);
    ASSERT_EQ(100, pixely);
}

TEST(TransformCoordinates, GlobaltoPixel_BR)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = 5;
    double Globaly = -5;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    
    
    int pixels = (int) mapSize / resolution_; 
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    double offset = (image_.cols/2);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(150, pixelx);
    ASSERT_EQ(150, pixely);
}

TEST(TransformCoordinates, GlobaltoPixel_R)
{
    
    cv::Mat image_;
    
    double mapSize=20.0;
    double Globalx = 5;
    double Globaly = 0;
    double resolution_ = 0.1;
    double robotPosex = 0;
    double robotPosey = 0;
    
    
    int pixels = (int) mapSize / resolution_; 
    image_ = cv::Mat::zeros(pixels,pixels, CV_8UC1);
    double offset = (image_.cols/2);
    Processing test;
    
    double pixelx = test.globaltoPixelx(image_, Globaly, robotPosey, resolution_, offset);
    double pixely = test.globaltoPixely(image_, Globalx, robotPosex, resolution_, offset);
    
    ASSERT_EQ(100, pixelx);
    ASSERT_EQ(150, pixely);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
