#include "rectangle.h"

void Rectangle::setWidthHeight(double w, double h)
{
    width_ = w;
    height_ = h;
}

double Rectangle::area(void)
{
 double resulta;
 resulta = width_*height_;
 return resulta;
}

double Rectangle::perimeter(void)
{
    double resultp;
    resultp = (2*width_)+(2*height_);
    return resultp;
}
