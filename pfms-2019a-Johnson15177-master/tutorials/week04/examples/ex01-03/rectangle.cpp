#include "rectangle.h"

Rectangle::Rectangle(double width, double height):
    Rectangle(0.0, 0.0, width, height)
{
    description_="point";
}

Rectangle::Rectangle(double centre_x, double centre_y, double width, double height):
    Shape(centre_x, centre_y), width_(width), height_(height)
{
}

void Rectangle::setWidthHeight(double width, double height)
{
    //!NOTES 
    // This is a example of why you should not allow direct access to member variables (why they are private)
    // Given we have a function to set the member varaibles, we also can leverage this function to set any
    // other member variables required, of perform any other operations that are needed to be executed
    // (such as invoking other methods)

    width_ = width;
    height_ = height;
    if (width_==height_) {
        description_ = "square";
    } else {
        description_ = "rectangle";
    }
}

double Rectangle::getArea(void)
{
    return width_ * height_;
}

double Rectangle::getPerimeter() {
    return 2 * (width_ + height_);
}

bool Rectangle::checkPoint(double x, double y)
{
    return  (x >= (centre_x_ - 0.5 * width_))
            && (x <= (centre_x_ + 0.5 * width_))
            && (y >= (centre_y_ - 0.5 * height_))
            && (y <= (centre_y_ + 0.5 * height_));
}

//#include <cmath>

//double dist = std::sqrt(std::pow(x - centreX_, 2) + std::pow(y - centreY_, 2));
//return dist < radius;
