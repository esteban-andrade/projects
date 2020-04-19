#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    Rectangle(double width, double height);
    Rectangle(double centre_x, double centre_y, double width, double height);
    void setWidthHeight(double width, double height);
    
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // A design to enable this is covered in when we introduce polymorphism
    double getArea (void);
    double getPerimeter();
    virtual bool checkPoint(double x, double y);
private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H