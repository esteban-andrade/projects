#include "triangle.h"

Triangle::Triangle()
    : width_(0.0), height_(0.0)
{
    description_ = "triangle";
}

Triangle::Triangle(double width, double height) : width_(width), height_(height)
{
    description_ = "isoc triangle";
}

void Triangle::setHeightWidth(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}
