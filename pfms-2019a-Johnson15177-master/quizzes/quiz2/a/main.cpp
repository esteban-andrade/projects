#include <iostream>
#include <vector>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"
#include "shape.h"
#include "circle.h"

using std::cout;
using std::endl;
using std::vector;

int main () {
<<<<<<< HEAD

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();    
    
=======
/*
>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
    Rectangle rectangle;
    rectangle.setHeightWidth(5.0, 3.5);
//    std::cout << "The area of box is " << rectangle.getArea() << std::endl;
//    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Triangle triangle(3.0, 4.0);
//    std::cout << "The area of our triangle is " << triangle.getArea() << std::endl;
//    std::cout << "It is a " << triangle.getDescription() << std::endl;

<<<<<<< HEAD
    Circle circle(5);

//
=======
    Circle circle(7);

>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
    vector<Shape*> shapes;
    shapes.push_back(&rectangle);
    shapes.push_back(&triangle);
    shapes.push_back(&circle);
<<<<<<< HEAD
    
=======

>>>>>>> c0839e28f5003efc4cf7d6a7fcec4d6577a36d5b
    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
    }
    */
    
}
