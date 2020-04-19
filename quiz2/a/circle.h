#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
  public:
    Circle();
    Circle(double diameter);
    void setDiameter(double diameter);
    double getArea();

  private:
    double diameter_; //Diameter of Circle
    double radius_;
};

#endif