Quiz 2
======

Part A
------

1) TASK: Modify the base class `Shape` file [shape.h](./a/shape.h) such that function getArea() is defined in `Shape`, and the child classes are required to implement this function.  
HINT: Polymorphism and the concept of virtual.

2) TASK: Create a Square and Traingle, and store both of them in a vector of type `Shape`

3) TASK: Add `Cicrle` as a child class of `Shape`.

4) TASK: Write a program that allows the user to specify `s` number of squares, `t` number of triangles, `c` number of cicrcles and `max_length` max length. Create the shapes with random lengths to be capped to `max_length`. 

5) TASK: Create a function that loops through shapes and display their area.
HINT: Think of the function signature.  


Part B
------

1) TASK: Modify the file [car.h](./a/car.h) so that it inherits behaviour from the base class, [controllerinterface](./a/controllerinterface.h).

2) TASK: Create an executable that instatiates two objects of type `Car` with different specifications and determine's their top speed.

3) QUESTION: When instantiating an object of the car class, which constructor(s) are caled and in what order.

4) TASK: Modify the code so that it accelerates both vehicles to top speed, deccelerates them to zero and determines the time difference between the two vehicles reaching zero.

5) QUESTION: Class [controllerinterface](./a/controllerinterface.h) is a special class,  and has special member functions, their syntax is replacing a definition with `=0`. What is the name of these special member functions and special class.




