Quiz 0
======

Part A
------
1) Attempt to compile the code in [folder a](./a). Why does the code fail to compile?
Because the identifier of two functions *printArray* and *printArray* are duplicated, which is caused the error "Redefinition of function *printArray* ".

2) Fix the compilation error. Why does a segmentation fault (segfault) occur?
When the array is created , its size is fixed, so we cannot expand the array by increase the index of the array. Furthermore, it is allocated to the memory within its fixed size, so when we exceed that size, we access to non-allocated memory space, which can cause segfault.

The way that I fix it is that allocating new bigger array, then assign current array and new random values to that new array. 

3) Modify the code so it will run

4) Create a function that prints only the elements of the array that are larger than : mean + one standard deviation

5) Create a function that assigns elements of array x to a vector named *vec* (HINT: decide on the correct type)

Part B
------
1) Look at the code in [folder b](./b). Implement the methods of the rectangle class in the [rectangle.cpp](./b/rectangle.cpp) file based on the definition provided in [rectangle.h](./b/rectangle.h)

2) Create an executable that uses the class to compute the area and perimeter of three rectangles

3) What do we call functions of a class?

Constructor: To initialise all class members.

Setter *set_value*: To set the value of members of the class.

Other member functions: To work with members of constructor. In detail:
+ *area* function: Return an area value of each instance.
+ *perimeter* function: Return an perimeter value of each instance.

4) What access specifiers are used?
Private access specifier is used for the constructor of the class so that the variables in the constructor cannot be accessed from outside of the class. Therefore they can only be called from other members of that same class.

All member functions of *Rectangle* class have public access specifier in order to be called in the *main* function.

5) What do we call variable *width_* in the *Rectangle* class?
It is called constructor parameters. It is only used inside the class.
