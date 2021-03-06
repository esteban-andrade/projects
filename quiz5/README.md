Quiz 5
======

Part A
------
1) TASK: Compile and execute the application [q5b_printer](./a/printer.cpp). After executing a few times you will notice that some of the messages appear jumbled. Apply a synchronisation primitive to prevent this from happening.

2) TASK: Compile and execute the application [q5b_fibonacci](./a/fibonacci.cpp). You will notice that the consumer thread attempts to get data faster than what the producer can create. Apply appropriate synchronisation primitives to ensure that the consumer waits for new data to arrive.

3) TASK: Compile and execute the application [q5b_generator](./a/generator.cpp). You will notice that either instantly, or after a period of time, the program hangs. What condition is causing the problem, and how?

4) QUESTION: What is a critical section?

5) QUESTION: How can you prevent race conditions from occurring?


Part B
------
1) TASK: Look at the code in [folder b](./b), and you will notice that a [library](./b/complex.h) for performing operations on complex numbers has been written. Compile and run the test [executable](./b/test_complex.cpp). The unit test `Divide` will fail. Fix the static method `divide()` in [complex.cpp](./b/complex.cpp) and verify that the test passes after re-compilation.

2) TASK: Write separate unit tests in [test_complex.cpp](./b/test_complex.cpp) for each function within [complex.cpp](./b/complex.cpp).

3) TASK: Make sure that you can compile and run the executable. Your tests should pick up a bug in the [complex library](./b/complex.cpp). Fix the bug and make sure all tests pass.

4) QUESTION: What are some of the benefits of unit testing?

5) QUESTION: Why would a single unit test, testing all the functions in the complex library, be generally considered bad practice?
