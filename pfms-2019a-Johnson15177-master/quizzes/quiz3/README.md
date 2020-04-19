Quiz 3
======

Part A
------

1) TASK: Fix the compilation error the code in [folder a](./a). HINT: What type of class is `vehicle` and why? it is the parent class as car and motorbike is inheriting from the vehicle class

2) TASK: In `main()` instansiate a vector of vehicles and implement printDetails function with an iterator and confirm compilation succeeds. **************done

3) TASK: Without using the attribute getters from the vehicle class, implement the `printDetails()` function in [main.cpp](./a/main.cpp). This function should print out vehicle specific information.
***************done

Part B
------

1) TASK: Create a function that accepts a deque and modifies it by adding user specified numbers of elements, each element is form a gaussian distribution (mean: 8, std dev 4) [deque_vector_ops.cpp](./b/deque_vector_ops.h)
*******done
2) We are now tasked to design a function to rearrange elements of out container (vector or deque) by bubble sort operation,refer for pseudo code  (https://en.wikipedia.org/wiki/Bubble_sort).
QUESTION: Which STL container is more suitable for the Bubble sort
**************done
3) TASK: Create a function that accept the chosen container and rearranges elements by bubble sort operation
***************done
4) TASK: Create a main to call functions 1, 2 and 3 and prints the container after each function.
*****************done
Part C
-------

Consider the code in [folder c](./c)  What value would you expect to see printed?

1) QUESTION: How many threads are running in parallel? 2

2) QUESTION: What is the specific problem with this code, causing it to fail? they're accessing the same "shared_int at the same time, causing it to fail

3) QUESTION:  Recommend one approach to fix the problem and outline merits of the solution?
make another "shared_int" such as "shared_int2" allowing it to operate separetly 


