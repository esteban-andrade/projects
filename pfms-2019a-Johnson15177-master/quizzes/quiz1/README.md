Quiz 2
======

Part A
------

1) QUESTION Attempt to compile the code in [folder a](./a). Why does it fail? (list the reasons)
- the main file did not define "person.h"
- "std::string name_;", "int age_" were in private when it should have been in public

2) TASK Fix the issue so that it compiles.

3) TASK Make the code more robust, with respect to "sane" values of age.

4) TASK Create a `crowrd` using a vector container of people, populate it with 3 people.

5) TASK Create a function that greets the oldest member of the `crowd`.

6) TASK Implement a safe guard to make the initialisation of `person` objects easier? (HINT: What special member function is missing in Person?)
- person object created and initialises name and age but conflicts with parts 4, 5
Person (std::string name, int age)
reference
person.h line 8
person.cpp line 5-12
main.cpp line 8, 14, 19
Part B
------

1) TASK Modify the file rectangle [rectangle](./b/rectangle.h) so it inherits from the base class of shape is [shape](./b/shape.h)

2) TASK Correct the missing access specifiers of base class [shape](./b/shape.h)

3) TASK Modify the main [main.cpp](./b/main.cpp) so that it creates a rectangle of size width=5 and height =3.5

4) QUESTION If you create a `Rectangle`, which constructor is called, and what does it initialise?
- the constructor called is the Rectangle() and it initialises the width and height

