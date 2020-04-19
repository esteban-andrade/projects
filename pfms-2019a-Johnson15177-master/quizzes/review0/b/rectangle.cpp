#include "rectangle.h"

void Rectangle::setWidthHeight(double w, double h){
    width_ = w;
    height_ = h;
}

double Rectangle::area(){
    return width_*height_;
}

double Rectangle::perimeter(){
    return width_ + height_;
}
