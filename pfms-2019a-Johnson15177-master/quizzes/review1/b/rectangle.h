#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle: public Shape // It now inherets the properties of shape, can be used freely
{
public:
    Rectangle();
    void setHeightWidth(double width, double height);
    
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // A design to enable this is covered in when we introduce polymorphism
    double getArea (void);
private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?
    //
    // They are in rectangle, to ensure that other relatives of shape cannot access them
    // E.g. if we had another class of "Triangle", we wouldn't want it accessing or tampering
    // with the values of "Rectangle". "Shape" containes only generic use functions and 
    // properties for use throghout all related classes.

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
