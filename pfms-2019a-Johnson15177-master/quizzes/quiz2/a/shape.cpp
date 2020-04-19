#include "shape.h"

Shape::Shape():
    description_("unknown shape")
{
}

void Shape::setCentre(double x, double y)
{
    centreX_=x;
    centreY_=y;
}


std::string Shape::getDescription()
{
    return description_;
}

void Shape::offset(double x, double y)
{
    centreX_ += x;
    centreY_ += y;
}
<<<<<<< HEAD
/*
Shape::RandNum():
    generator_(seed), distribution_(0.0, 10)
{
}
*/
=======

void Shape::MaxLength(int seed):
    generator_(seed), distribution_(0.0, Max_Length_)
{
}

>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
