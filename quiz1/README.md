Quiz 2
======

Part A
------

1) QUESTION Attempt to compile the code in [folder a](./a). Why does it fail? (list the reasons)
Did not include the person header file in the main src file.
setName function & setAge function was coded corresponding to their signatures.

2) TASK Fix the issue so that it compiles. Done

3) TASK Make the code more robust, with respect to "sane" values of age.DONE

4) TASK Create a `crowd` using a vector container of people, populate it with 3 people. DONE

5) TASK Create a function that greets the oldest member of the `crowd`.

6) TASK Implement a safe guard to make the initialisation of `person` objects easier? (HINT: What special member function is missing in Person?)
By creating a default constructor for the Person class will allow it to initialise with just one function and two signatures instead of making 2 functions which
gives less code to write.

Part B
------

1) TASK Modify the file rectangle [rectangle](./b/rectangle.h) so it inherits from the base class of shape is [shape](./b/shape.h)
DONE

2) TASK Correct the missing access specifiers of base class [shape](./b/shape.h)

3) TASK Modify the main [main.cpp](./b/main.cpp) so that it creates a rectangle of size width=5 and height =3.5
DONE

4) QUESTION If you create a `Rectangle`, which constructor is called, and what does it initialise?
It initialises the default contructor with no signatures or arguments.
