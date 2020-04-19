#include "processing.h"
#include <image_transport/image_transport.h>

double Processing::pixeltoGlobalx(cv::Mat image_, double pixely, double robotPosex, double resolution_)
{
    double Globalx = ((pixely - (image_.cols/2)) * resolution_) + robotPosex;
    
    return Globalx;
}

double Processing::pixeltoGlobaly(cv::Mat image_, double pixelx, double robotPosey, double resolution_)
{
    double Globaly = -((pixelx - (image_.rows/2)) * resolution_) + robotPosey;
    
    return Globaly;
}

double Processing::globaltoPixelx(cv::Mat image_, double Globaly, double robotPosey, double resolution_, double offset)
{
    
    int pixelx = abs(((Globaly - robotPosey)/resolution_) - offset);
    
    return pixelx;
}

double Processing::globaltoPixely(cv::Mat image_, double Globalx, double robotPosex, double resolution_, double offset)
{
    int pixely = ((Globalx - robotPosex)/resolution_) + offset;
    
    return pixely;
}
