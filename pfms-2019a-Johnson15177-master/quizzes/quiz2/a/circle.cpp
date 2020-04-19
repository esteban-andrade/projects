#include "circle.h"

Circle::Circle(double radius):
    radius_(radius)
{
<<<<<<< HEAD
    description_="cicle";
}


=======
    description_ = "circle";
}

>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getArea()
{
<<<<<<< HEAD
    return 3.14159265358979323846* (radius_ * radius_);
=======
    return (3.14159*(radius_*radius_));
>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
}
