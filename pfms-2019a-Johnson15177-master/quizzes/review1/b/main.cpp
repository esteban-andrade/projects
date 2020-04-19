#include <iostream>
#include "rectangle.h"


int main () {
    //! TODO: Create a rectangle
	Rectangle rectangle;
	
	rectangle.setHeightWidth(5, 3.5);

    // Print some info about it
    std::cout << "The area of box is " << rectangle.getArea() << std::endl;
    std::cout << "It is a " << rectangle.getDescription() << std::endl;

	//QUESTION ANSWER
	// it calls the rectangle constructor, because the class of 'rectangle'
	// is fundamentally of the class "Rectangle". This class only inherets the
	// ability to call on the PUBLIC and PROTECTED functions and values contained in
	// the class "Shape".
	//
	// NOTE: PRIVATE elements of a class are not inhereted, private to the class itself.
	//       To 
	
    //!ADDITIONAL QUESTIONS TO CONSIDER
    // - Would it make sense to create an instance of shape?
    //      Not really, shape does not hold any values of use, it only holdes values that
    //      mean something in the context of the other classes.
    //
    // - A design to prohibit this is covered when we introduce polymorphism

}
