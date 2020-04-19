#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"

class Circle : public Shape
{
public:
    Circle(double radius);
    void setRadius(double radius);
<<<<<<< HEAD
    double getArea ();
private:
    double radius_;    


=======
    double getArea();

private:
    double radius_;
>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
};

#endif // CIRCLE_H
