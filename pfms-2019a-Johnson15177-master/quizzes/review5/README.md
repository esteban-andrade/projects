Quiz 5
======

Part A
------
1) TASK: Compile and execute the application [q5b_printer](./a/printer.cpp). After executing a few times you will notice that some of the messages appear jumbled. Apply a synchronisation primitive to prevent this from happening.

2) TASK: Compile and execute the application [q5b_fibonacci](./a/fibonacci.cpp). You will notice that the consumer thread attempts to get data faster than what the producer can create. Apply appropriate synchronisation primitives to ensure that the consumer waits for new data to arrive.

3) TASK: Compile and execute the application [q5b_generator](./a/generator.cpp). You will notice that either instantly, or after a period of time, the program hangs. What condition is causing the problem, and how?

4) QUESTION: What is a critical section? A critical section is a section of code that must be ran serially due to it sharing data or accessing data that other threads may want to also access at the same time. If multiple threads try to access this data concurrently then a data race could occur and output will not be as expected.

5) QUESTION: How can you prevent race conditions from occurring? Race conditions can be prevented by locking a shared mutex to mutually exclude critical sections of code that may alter or access data which other threads could also be wanting access to.


Part B
------
1) TASK: Look at the code in [folder b](./b), and you will notice that a [library](./b/complex.h) for performing operations on complex numbers has been written. Compile and run the test [executable](./b/test_complex.cpp). The unit test `Divide` will fail. Fix the static method `divide()` in [complex.cpp](./b/complex.cpp) and verify that the test passes after re-compilation.

2) TASK: Write separate unit tests in [test_complex.cpp](./b/test_complex.cpp) for each function within [complex.cpp](./b/complex.cpp).

3) TASK: Make sure that you can compile and run the executable. Your tests should pick up a bug in the [complex library](./b/complex.cpp). Fix the bug and make sure all tests pass.

4) QUESTION: What are some of the benefits of unit testing? Unit testing allows us to quickly test if our code runs as intended and can be used to debug our code by testing code which our final output may rely on. It is also useful for when you are working in a team and you want to verify quickly if your additions to your section in a group project may alter someone elses section in the project, which may cause a long process of debugging that can be easily avoided through unit testing.

5) QUESTION: Why would a single unit test, testing all the functions in the complex library, be generally considered bad practice? This is generally bad practice as your unit tests should be grouped in tests that are relevant to other groups of code. If you group your tests it is much easier to navigate and see the results of your tests when they fail or pass.
