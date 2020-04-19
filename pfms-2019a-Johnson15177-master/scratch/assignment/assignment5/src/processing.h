#ifndef PROCESSING_H
#define PROCESSING_H

#include <opencv2/opencv.hpp>

class Processing {

public:
      double pixeltoGlobalx(cv::Mat image_, double pixely, double robotPosex, double resolution_);
      
      double pixeltoGlobaly(cv::Mat image_, double pixelx, double robotPosey, double resolution_);
      
      double globaltoPixelx(cv::Mat image_, double Globaly, double robotPosey, double resolution_, double offset);
      
      double globaltoPixely(cv::Mat image_, double Globalx, double robotPosex, double resolution_, double offset);
};

#endif // PROCESSING_H
