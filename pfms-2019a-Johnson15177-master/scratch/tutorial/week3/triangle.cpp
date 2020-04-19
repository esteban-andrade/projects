#include "triangle.h"

Triangle::Triangle() {
}

void Triangle::setBaseHeight(double base, double height)
{
    base_ = base;
    height_ = height;
}

double Triangle::getArea(void)
{
    return((base_ * height_)/2);
}
