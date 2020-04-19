#ifndef TRIANGLE_H
#define TRIANGLE_H
#include <iostream>
#include "shape.h"

class Triangle : public Shape
{
public:
    Triangle();
    void setBaseHeight(double base, double height);
    double getArea (void);
    
private:
    double base_;
    double height_;
};

#endif // TRIANGLE_H
