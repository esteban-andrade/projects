#include <iostream>
#include <vector>

#include "rectangle.h"
#include "triangle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

int main () {
    vector<shape*> shapes;
    
    Rectangle r1(5.0, 5.0), r2(3.0, 4.0);
    Triangle t1(3.0, 4.0), t2(3.0, 7.0);
    
    shapes.push_back(&r1);
    shapes.push_back(&r2);
    shapes.push_back(&t1);
    shapes.puch_back(&t2);
    
    for (long int num=0; num<5; num++) {
        std::cout << "shape #" << num << std::endl;
        shapes.push_back(new Rectangle(2.0*num, 4.0*num));
    }
    
    double total_area = 0.0;
    
    double
    
 //  rectangle.setHeightWidth(5.0, 3.5);
//    std::cout << "The area of box is " << rectangle.getArea() << std::endl;
//    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    
//    std::cout << "The area of our triangle is " << triangle.getArea() << std::endl;
//    std::cout << "It is a " << triangle.getDescription() << std::endl;
    
    Shape *square = &rectangle;
    Shape *tri = &triangle;
    
    square->getArea();
    tri->getArea();
    
   
    
    vector<Shape*> shapes;
    shapes.push_back(&rectangle);
    shapes.push_back(&triangle);

    for (auto s : shapes) {
        cout << s->getDescription() << " has area " <<  (*s).getArea() << endl;
    }   
    
}
