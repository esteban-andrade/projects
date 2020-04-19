#include <iostream>
#include <cmath>

class Rectangle {
private:
    double width_;
    double height_;
public:
    void setWidthHeight(double w, double h);
    double area(void);
    double perimeter(void);
};
