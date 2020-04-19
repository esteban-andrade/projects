#include "rectangle.h"

Rectangle::Rectangle(double width, double height):
width_(width), height_(height)
{
    if (width_==height_) {
        description_ = "square";
    } else {
        description_ = "rectangle";
    }
}

void Rectangle::setHeightWidth(double width, double height)
{
    //!NOTES 
    // This is a example of why you should not allow direct access to member variables (why they are private)
    // Given we have a function to set the member varaibles, we also can leverage this function to set any
    // other member variables required, of perform any other operations that are needed to be executed
    // (such as invoking other methods)

    width_ = width;
    height_ = height;
    
}

double Rectangle::getArea(void)
{
    //std::cout << "in the rectangle class" << std::endl;
    return width_ * height_;
}

