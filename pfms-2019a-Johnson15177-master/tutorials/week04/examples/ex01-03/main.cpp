#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <array>

#include "rectangle.h"
#include "triangle.h"
#include "shape.h"

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::array;

//void vector_way()
//{
//    vector<Shape*> shapes;

//    const int num_rectangles = 2;
//    const int num_triangles = 2;

//    // Create rectangles
//    array<Rectangle, num_rectangles> rectangles;
//    for (auto &r : rectangles) {
//        r.setHeightWidth(2.0, 3.0);
//        shapes.push_back(&r);
//    }

//    array<Triangle, num_triangles> triangles;
//    for (auto &t : triangles) {
//        t.setHeightWidth(4.0, 5.0);
//        shapes.push_back(&t);
//    }

//    double total_area = 0.0;
//    for (auto s : shapes) {
//        total_area += s->getArea();
//    }

//    cout << "Total area is " << total_area << endl;

//}

//void new_way()
//{
//    vector<Shape*> shapes;

//    int num_rectangle = 100;
//    int num_triangles = 100;

//    // Create rectangles
//    for (int i = 0; i < num_rectangle; i++) {
//        shapes.push_back(new Rectangle(3.0, 4.0));
//    }

//    for (int i = 0; i < num_triangles; i++) {
//        shapes.push_back(new Triangle(4.0, 6.0));
//    }

//    double total_area = 0.0;
//    for (auto s : shapes) {
//        total_area += s->getArea();
//    }

//    cout << "Total area is " << total_area << endl;

//    for (auto s : shapes) {
//        delete s;
//    }





int main () {
    vector<Shape*> shapes;

    //OPTION 1 - Pushing back the pointers of objects (reference get's the address)
    Rectangle r1(2.0, 4.0), r2(3.0, 4.0);
    Triangle t1(6.0, 4.0), t2(3.0, 7.0);

    shapes.push_back(&r1);
    shapes.push_back(&r2);
    shapes.push_back(&t1);
    shapes.push_back(&t2);

    //OPTION 2 - Using new to generate the pointer
    // In example below the sides are not random, rather set to be 2*num and 4*num
    // This will require a delete to avoid memory leaks, 
    for(long int num=0;num<5;num++){
      std::cout << "shape #" << num << std::endl;
      shapes.push_back(new Rectangle(2.0*num,4.0*num));
    }


   double total_area = 0.0;

   double x, y;
   cout << "Enter an x and y coordinate separated by spaces: ";
   cin >> x >> y;

   for (auto s : shapes) {
       if (s->checkPoint(x, y)) {
           total_area += s->getArea();
       }
   }

   cout << "Total area of shapes overlapping this point is " << total_area << endl;
}
