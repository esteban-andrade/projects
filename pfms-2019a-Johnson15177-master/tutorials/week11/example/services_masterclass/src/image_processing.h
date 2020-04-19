#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>

class ImageProcessing {

public:
  /*! @brief Checks wether the origin and destination can be connected with a line, such that line only goes over free space
   *
   *  @param[in]    cv::Point oirgin - point of origin [i,j] in image coordinates
   *  @param[in]    cv::Point destination - point of destination in image coordinates
   *  @return bool  The points can be connected with a line which only goes over free space
   */
    bool checkConnectivity(cv::Mat image, cv::Point origin, cv::Point destination);
};

#endif // IMAGE_PROCESSING_H
