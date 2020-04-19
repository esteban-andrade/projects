#include "image_processing.h"
#include <image_transport/image_transport.h>

bool ImageProcessing::checkConnectivity(cv::Mat image, cv::Point origin, cv::Point destination)
{
  // BEGIN
  // Following visualisation code (to END label) is only enabled for the tutorial
  // To understand the inner working.
  // When delivering the code as a library it should be removed

  // Create an 8-bit 3-channel image from input image, so we can draw on it
  cv::Mat color_image;
  cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
  // Draw the origin in Green (the colors are BGR so G is 255 and other two channels zero)
  cv::circle(color_image,origin,1,cv::Scalar(0,255,0),2);
  // Draw destination in Red
  cv::circle(color_image,destination,1,cv::Scalar(0,0,255),2);
  // Let's put some text, just for additional understanding
  cv::putText(color_image, "green - origin ,red - destination", cv::Point(0,10), cv::FONT_HERSHEY_PLAIN, 0.7, cv::Scalar(255,0,0));
  // Make a window called test, which can be adjusted in size
  cv::namedWindow("test",cv::WINDOW_NORMAL);
  // Draw the color image on the windows called "test"
  cv::imshow("test",color_image);
  // wiat for a maximum of 5000ms for a user to click on window, then close window
  cv::waitKey(5000);
  // END


  bool connectivity = true;

  //! Create a Line Iterator  between origin and destination
  cv::LineIterator lineIterator(image, origin, destination);

  //! @todo Ex01 : Insert code to search the line for non-white pixel
  //! If there is a non-white pixel it means the robot would not be able to
  //! Travel over the line origin and destination
  //!
  //! code needed here ---
  //! Look at Week 08 tutorial examples - drawing function
    
    for(int i=0; i<lineIterator.count; i++, lineIterator++){
        uchar *pixel = (uchar*)*lineIterator;
        if(*pixel == 255){
            //connectivity = true;
        }
        else {
            connectivity = false;
        }
    }

  return connectivity;

}
