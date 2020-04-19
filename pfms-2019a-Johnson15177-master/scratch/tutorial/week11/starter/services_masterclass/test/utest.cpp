#include <gtest/gtest.h>
#include <climits>

#include <ros/package.h> //This tool allows to identify the path of the package on your system

//#include <cv_bridge/cv_bridge.h>
#include "../src/image_processing.h"


TEST(GeneratedImage,ConnectionFree){

    //! The below code tests line connectivity
    //! On a occupied, free, and partial free OgMap

    double mapSize=20.0;
    double resolution=0.1;

    int pixels = (int) mapSize / resolution;

    //! Create an object of image processing as we will use the public function of that object to run tests against
    ImageProcessing imageProcessing;

    // Create an OgMap which is occupied (all zeros)
    cv::Mat image = cv::Mat::zeros(pixels,pixels, CV_8UC1);

    // Let's check map size compared to allocation, just in case
    ASSERT_EQ(pixels, image.rows);
    ASSERT_EQ(pixels, image.cols);

    // Let's check the map is allocated all zeros
    ASSERT_EQ(0,image.at<uchar>(0,0));

    // o------> j
    // |
    // |
    // v i
    //

    cv::Point orig(pixels/4,pixels/2);
    cv::Point dest(3*pixels/4,pixels/2);

    // Should the image allow the origin and destination to be connected in a straight line?
    ASSERT_FALSE(imageProcessing.checkConnectivity(image,orig,dest));


    //Create an OgMap which is free space
    image.setTo(cv::Scalar(255,255,255));

    // Does the image now allow the origin and destination to be connected with a straight line?
    ASSERT_TRUE(imageProcessing.checkConnectivity(image,orig,dest));

    // Draw a black line down the middle
    cv::Point pt1(pixels/2,0);
    cv::Point pt2(pixels/2,pixels);
    cv::line(image,pt1,pt2,cv::Scalar(0,0,0),1);

    // Does the image now allow the origin and destination to be connected with a straight line?
    ASSERT_FALSE(imageProcessing.checkConnectivity(image,orig,dest));

}

TEST(LoadedImage,ConnectionFree){

    //! The below code tests line connectivity
    //! On a loaded image

    //! Create an object of image processing as we will use the public function of that object to run tests against
    ImageProcessing imageProcessing;

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("week11");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/samples/";
    // The file is called smiley_og.png
    std::string file = path + "image.png";

    cv::Mat image = cv::imread(file,0);//The zero forces it to a grayscale image (single channel, such as OgMap)

    ASSERT_EQ(99, image.cols);// Test if aquare image
    ASSERT_EQ(image.channels(),1);    // Test if aingle channel

    //Questions? Why do we have the {} around the below statements?
    {
      cv::Point org(1*image.cols/4,image.rows/2);
      cv::Point dest(3*image.cols/4,image.rows/2);

      ASSERT_TRUE(imageProcessing.checkConnectivity(image,org,dest));
    }

    {
      cv::Point org(image.rows/2,1*image.cols/4);
      cv::Point dest(image.rows/2,3*image.cols/4);

      ASSERT_FALSE(imageProcessing.checkConnectivity(image,org,dest));
    }

    //! @todo Ex02 : Insert code to load a image of a OgMap and search a line between the centre of the OgMap and a point
    //!


}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
