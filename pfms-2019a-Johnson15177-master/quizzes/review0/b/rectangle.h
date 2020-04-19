#include <iostream> // Includes std::cout and friends so we can output to console
#include <cmath>   // Includes the math library

class Rectangle{
    // We use private that is access specifier for the constructor
private:
    double width_;
    double height_;
    // We use public that is access specifier for function used by rectangle_main
public:
    void setWidthHeight(double w, double h);
    double area(void);
    double perimeter(void);
};
