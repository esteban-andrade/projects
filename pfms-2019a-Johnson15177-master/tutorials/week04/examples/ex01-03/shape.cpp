#include "shape.h"

Shape::Shape(double centre_x, double centre_y):
    centre_x_(centre_x),
    centre_y_(centre_y),
    description_("unknown shape")
{
}

Shape::~Shape()
{

}

void Shape::setCentre(double x, double y)
{
    centre_x_=x;
    centre_y_=y;
}


std::string Shape::getDescription()
{
    return description_;
}

void Shape::offset(double x, double y)
{
    centre_x_ += x;
    centre_y_ += y;
}

