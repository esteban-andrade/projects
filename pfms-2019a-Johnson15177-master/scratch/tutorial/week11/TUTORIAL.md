Week 11 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

This week's tutorial questions will make use of the [a5_support] package provided to assist you with assignment 5.
The tutorial extends the pre-work [quiz6a].

Before you get started make sure you do the following:

* Check out the latest code from the repository
* Link the [week11] package to your catkin workspace, (ie if your [services_masterclass] folder path is /home/student/git/pfms-2019a-<YOURHANDLE>/tutorial/week11/services_masterclass then execute:
```bash
cd ~/catkin_ws/src
ln -s /home/student/git/pfms-2019a-<YOURHANDLE>/tutorial/week11/services_masterclass
```

* Compile packages using `catkin_make` and `catkin_make tests`
```bash
cd ~/catkin_ws
catkin_make
catkin_make tests
```

Ex01: GoogleTest in ROS (Unit Testing)
----------------------------

The [week11] package defines some unit tests and compiles a test executable called `week11-test`.

Unit tests are generally performed on libraries, code that is well defined and is often isolated from the ROS framework of topics / services. In this case we have a [ImageProcessing] class that is compiled into a library and thereafter used within the ROS framework.

The test cases are done in isolation from ROS, either create a OpenCV image (cv::Mat) with a particular apperance, or load a image and test connectivity between a origin point and a destination point. By connectivity, we test wether a straight line between the origin and destination will solely go over free space (line only crosses white pixels which represent free space).

It is the nature of unit tests, that the answer is known for the test, checking the function for a variety of situations with known answer. Some of the test cases fail.

You can run tests from within `catkin_ws` directory
```bash
cd ~/catkin_ws
./devel/lib/week11/week11-test
```

Your TASK is to:
* Determine which function is being called from the test cases
* The function is not completely implemented, using your knowledge of OpenCV from week 08 (drawing_functions), complete the code, so it returns false if the line between origin and destination goes over any pixel that is NOT free space.


Ex02: GoogleTest in ROS (Unit Testing) - Adding Additional Image for Testing
----------------------------

Building on Ex01

TASK: Save a realistic OgMap image with areas of free space, unknown space, and occupied space and update [utest.cpp] to load and test against this image.

Ex03: GoogleTest in ROS (Unit Testing) - Adding Additional Code for Testing
----------------------------

TASK: Add your code from [quiz6a] that checks wether a point is free space to the [ImageProcessing] class and create a Unit test for it.

Ex04: Using Services in ROS + Checking if a Point is Reachable
----------------------------

In your node and the [PfmsSample] class, *advertise* a service named `face_goal` with the type `a5_setup::RequestGoal`.
The pose supplied on this request has an x and y value.

TASK: Adjust the code so it returns a `true` in the acknowledgment of the service call if the point can be reached in a straight line from current robot pose, only traversing free space.

To call the service
```bash
rosservice call /face_goal "x: 0.0
y: 0.0
theta: 0.0"
```

Ex05: Using Services in ROS + Controlling the Robot
----------------------------

The robot is controlled via a `geometry_msgs::Twist` and the control has two component `cmdvel.linear.x` which is the linear velocity and `cmdvel.angular.z`, the angular velocity (or V, Omega as we have used in Assignment 3 - chasing the bogey)

TASK: Control the robot so it turns to face the point requested (and does not move forward). Yor node should also allow for any new requests to overwrite the current actioned goal.

### Questions to assist ###

* Given we need to respond to service, what do we need to setup and where?
* Given we need to control the robot to turn and stop at a specific angle, what type of control do you envisage?
* What do we need to compute (is it a one off or continous task) to enable the control?
* At what rate does the control need to work?
* In what method(s) of the PfmsSample should we complete the control?


[ImageProcessing]: services_masterclass/src/sample.h
[utest.cpp]: services_masterclass/test/utest.cpp
[quiz6a]: ../../quizzes/quiz6/a
[a5_support]: ../../skeleton/a5-support
