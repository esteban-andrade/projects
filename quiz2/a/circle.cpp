#include "circle.h"

Circle::Circle()
    : diameter_(0.0)
{
    description_ = "Circle";
}

Circle::Circle(double diameter)
    :diameter_(diameter), radius_(diameter * 0.5)
{
    description_ = "Circle";
}

void Circle::setDiameter(double diameter)
{
    diameter_ = diameter;
    radius_ = diameter * 0.5;
}

double Circle::getArea()
{
    return 3.14 * radius_ * radius_;
}