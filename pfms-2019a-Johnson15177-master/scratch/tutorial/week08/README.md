
Week 8 Tutorial Questions
=========================
Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

We will use Google's C++ unit testing framework, [googletest]. A good starting point for using googletest is to read the [primer] document. Also see the provided [unit_testing_examples] in the starter directory.


Unit Testing : Simple Exaple - Ex01
-------------------

We have creating a simple library, we add some **unit tests** to make sure they are working as intended. 

1. We have googletest added into the project directory via the lib/gtest folder
2. We have Modified CMakeLists.txt to build and link against googletest
3. We have a testing executable which tests the correct functionality of your classes using the macros provided by google test. 

Run the unit test.

Why does the test fial, can we fix it?

[unit_testing_examples]: ./starter/unit_testing
[googletest]: https://github.com/google/googletest
[primer]: https://github.com/google/googletest/blob/master/googletest/docs/primer.md

Unit Testing : More complex library - Ex02
-------------------------

For this exercise we have provided some code written for (Assigment 2). Test if the code passes the unit test.
Now, add another unit test to check that the laser angular resolution only accepts the allowable resolutions.

Unit Testing : Testing your assignment 2 library - Ex03
-------------------------

Using code form Ex02, copy your ranger.h , ranger.cpp and the laser/radar files.
Run the unit tests against these files, do they work? 
What do you need to modify to get your code to work.

**Outside of class, contemplate how you could test your Assignment 2 fusion**

OpenCV : Samples 
-------------------
 
We have provided two complete examples [access_pixels] and [display_image].

The [display_image] example loads an image supplied as a parameter on the command line. Download an image, maybe [grumpy_cat] and give it a go.

Example two [access_pixels] demonstrates how to access individual pixels of an image. The key here is line `unsigned char &pixel = image.at<unsigned char>(i, j)`, what does the `unsigned char` mean with respect to `image`? 


OpenCV : Writing code to search along a line in an image
-------------------

We develop a very basic ray tracing example, search between two points in an image for presence of a bluw pixel along the line joining these two points.
Finish the code, to detect if a line between two points collides with a blue pixel using a [line_iterator] for searching.

[access_pixels]: ./starter/opencv/access_pixels
[display_image]: ./starter/opencv/display_image
[drawing_functions]: ./starter/opencv/drawing_functions
[gumpy_cat]: https://www.google.com/search?tbm=isch&as_q=grumpy_cat&tbs=isz:lt,islt:4mp,sur:fmc
[line_iterator]: https://docs.opencv.org/3.1.0/dc/dd2/classcv_1_1LineIterator.html

