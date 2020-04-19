// We need to include the declaration of our new rectangle class in order to use it.
#include "rectangle.h"

#include <iostream>

int main () {

    // Create a rectangle object
    Rectangle rectangle;
    Rectangle rectangle1;
    Rectangle rectangle2;

    // Set the values of the sides
    rectangle.setWidthHeight(5.0,5.0);
    rectangle1.setWidthHeight(6, 10);
    rectangle2.setWidthHeight(10, 3);

    // Get the area and print it to screen
    double result = rectangle.area();
    std::cout << "The area of rectangle is " <<
    std::cout << result << std::endl;
    std::cout << rectangle1.area() << std::endl;
    std::cout << rectangle2.area() <<std::endl;

    // get the perimeter and print to screen
    double perim = rectangle.perimeter();
    std::cout << perim <<std::endl;
    return 0;
}
