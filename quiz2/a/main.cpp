#include <iostream>
#include <vector>
#include <random>
#include <chrono>

#include "rectangle.h"
#include "triangle.h"
#include "circle.h"
#include "shape.h"

using std::cout;
using std::endl;
using std::vector;

double randomNumber(double maxLength)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<> value_distribution(0 , maxLength);
    return value_distribution(generator);
}

int main()
{
    int s;
    int t;
    int c;
    double maxLength;
    Rectangle rectangle;
    rectangle.setHeightWidth(5.0, 3.5);
    //    std::cout << "The area of box is " << rectangle.getArea() << std::endl;
    //    std::cout << "It is a " << rectangle.getDescription() << std::endl;

    Triangle triangle(3.0, 4.0);
    //    std::cout << "The area of our triangle is " << triangle.getArea() << std::endl;
    //    std::cout << "It is a " << triangle.getDescription() << std::endl;

    Rectangle square;
    square.setHeightWidth(3.0, 3.0);

    std::cout << "How many squares would you like?" << std::endl;
    std::cin >> s;
    std::cout << "How many Triangles would you like?" << std::endl;
    std::cin >> t;
    std::cout << "How many Circles would you like?" << std::endl;
    std::cin >> c;
    std::cout << "What is the max length for the shapes?" << std::endl;
    std::cin >> maxLength;

    vector<Shape *> Shapes;

    for (int i = 0; i < s; i++)
    {
        Rectangle *square = new Rectangle;
        square->setHeightWidth(randomNumber(maxLength), randomNumber(maxLength));
        Shapes.push_back(square);
    }

    for (int i = 0; i < t; i++){
        Triangle *triangle = new Triangle;
        triangle->setHeightWidth(randomNumber(maxLength), randomNumber(maxLength));
        Shapes.push_back(triangle);
    }

    for (int i = 0; i < c; i++){
        Circle *circle = new Circle;
        circle->setDiameter(randomNumber(maxLength));
        Shapes.push_back(circle);
    }

    for (auto s : Shapes)
    {
        cout << s->getDescription() << " has area " << (*s).getArea() << endl;
    }
}
