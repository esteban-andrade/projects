#include "triangle.h"
#include <cmath>

Triangle::Triangle(double width, double height):
    Triangle(0.0, 0.0, width, height)
{
    description_ = "isoc triangle";
    updateEdges();
}

Triangle::Triangle(double centre_x, double centre_y, double width, double height):
    Shape(centre_x, centre_y), width_(width), height_(height)
{
}

void Triangle::setCentre(double x, double y)
{
    Shape::setCentre(x, y);
    updateEdges();
}

void Triangle::offset(double x, double y)
{
    Shape::offset(x, y);
    updateEdges();
}

void Triangle::setWidthHeight(double width, double height)
{
    width_ = width;
    height_ = height;
    updateEdges();
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}

double Triangle::getPerimeter()
{
    double hyp = std::sqrt(std::pow(height_, 2) + std::pow(0.5 * width_, 2));
    return width_ + 2 * hyp;
}

bool Triangle::checkPoint(double x, double y)
{
    return base_edge_.pointAboveLine(x, y)
           && !left_edge_.pointAboveLine(x, y)
           && !right_edge_.pointAboveLine(x, y);
}

void Triangle::updateEdges()
{
    base_edge_.setGradient(0.0);
    base_edge_.setYIntercept(centre_y_ - 0.5 * height_);
    left_edge_.fromPoints(centre_x_ - 0.5 * width_, centre_y_ - 0.5 * height_,
                          centre_x_,                centre_y_ + 0.5 * height_);
    right_edge_.fromPoints(centre_x_ + 0.5 * width_, centre_y_ - 0.5 * height_,
                          centre_x_,                centre_y_ + 0.5 * height_);
}
