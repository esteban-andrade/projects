#include "rectangle.h"
#include <iostream> // Includes std::cout and friends so we can output to console
#include <cmath>   // Includes the math library

void Rectangle::setWidthHeight(int x, int y) {
    Rectangle::width_ = x;
    Rectangle::height_ = y;
}

int Rectangle::area() {
    return(width_*height_);
}

int Rectangle::perimeter() {
    return 2*(width_+height_);
}
