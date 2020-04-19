#include "rectangle.h"

#include <iostream>

int main()
{
    Rectangle rectangle;
    Rectangle rectangle1;
    Rectangle rectangle2;
    
    rectangle.setWidthHeight(5.0,5.0);
    rectangle1.setWidthHeight(10.0,5.0);
    rectangle2.setWidthHeight(6.0,15.0);
    
    double area = rectangle.area();
    double area1 = rectangle1.area();
    double area2 = rectangle2.area();
    
    std::cout << "area 1 is " << area << std::endl;
    std::cout << "area 2 is " << area1 <<std::endl;
    std::cout << "area 3 is " << area2 <<std::endl;
    
    double perim = rectangle.perimeter();
    double perim1 = rectangle1.perimeter();
    double perim2 = rectangle2.perimeter();
    
    std::cout << "perimeter 1 is " << perim << std::endl;
    std::cout << "perimeter 2 is " << perim1 << std::endl;
    std::cout << "perimeter 3 os " << perim2 << std::endl;
    
    return 0;
}
