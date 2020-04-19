// We need to include the declaration of our new rectangle class in order to use it.
#include "rectangle.h"

#include <iostream>

int main () {

    // Create rectangle objects
    Rectangle rectangle;
    Rectangle rectangle1;
    Rectangle rectangle2;

    // Set the values of the sides
    rectangle.setWidthHeight(5.0,5.0);
    rectangle1.setWidthHeight(10.0,5.0);
    rectangle2.setWidthHeight(6.0,15.0);


    // Get the area and print it to screen
    double area = rectangle.area();
    double area1 = rectangle1.area();
    double area2 = rectangle2.area();

    std::cout << area << std::endl;
    std::cout << area1 << std::endl;
    std::cout << area2 << std::endl;

    // Get the perimeter and print it to screen
    double perm = rectangle.perimeter();
    double perm1 = rectangle1.perimeter();
    double perm2 = rectangle2.perimeter();

    std::cout << perm << std::endl;
    std::cout << perm1 << std::endl;
    std::cout << perm2 << std::endl;






    return 0;
}
