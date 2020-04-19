Week 2 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

Classes - Ex01
--------------------
Create a class for generating an array of random numbers (doubles)

The class should have:
* A constructor that accepts
    * A seed for a Random Number Generator 
    * A number N - specifying how many random numbers are generated each time
* Have a Function That
    * Accepts an array and populates the array with N numbers generated at each time 

Write a program that uses this function to populate an array of numbers

Hints:

[Random Generator](http://www.cplusplus.com/reference/random/uniform_real_distribution/)

Questions:
* What are the pinch points here, how to ensure the program does not cause a segmentation fault?
* What is a segmentation fault?
* Why do we have N in constructor?
* Who is responsible for defining the max length of the array?
* Is there any way to make this allocation dynamic?


Classes - Ex02
------------------
Write a: 
* function that accepts an Array of Rectangles and returns the combined area of all Rectangles in the array.
* program that creates an Array of Rectangles with random sides and uses the function to computes the combined area.

Hints:

Use the quiz00 material and previous example to start solving this problem
