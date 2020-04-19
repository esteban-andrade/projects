Week 9 Tutorial Questions
=========================

Work through these questions and make sure you understand what is going on in each example. If you have any questions about the material please raise them in the next tutorial session.

Ex01: Getting Started with ROS
-------------------------------

For the remainder of the semester we will be working with ROS. Complete the following steps to help you get started with ROS:

* If you plan to use your own computer we recommend installing ROS Kinetic Kame on Ubuntu 16.04 by following the official [ROS Installation Instructions].
* We also recommend going through all 20 beginner level [ROS Tutorials]
* At a minimum we expect you to complete tutorials 1-6 for this lesson 
* At the completion of these tutorials you should have a catkin workspace in your home directory i.e. `~/catkin_ws` and a line in your `~/.bashrc` file to *source* it.
```bash
source /home/<username>/catkin_ws/devel/setup.bash
```


Ex02: Building and Running the Starter Package 
-----------------------------------------------

Now we will take a ROS Node and modify its code to make it do something new. This should help you get familiar with writing software for ROS.

* If you have a `beginner_tutorials` package in your workspace either delete it or hide it by executing `touch CATKIN_IGNORE` from inside the package directory. If you don't do this it will clash with the starter package provided
* A boiler plate package is provided in the [starter](./starter) directory. To use it either copy or link it to your workspace. Link makes any changes reflected in the original folder and able to be backed up on git (so it has advantages). The link (symbolic link used here) in Linux is shortcut in Windows.
To copy:
```bash
cp -r path-to-this-folder/starter/beginner_tutorials ~/catkin_ws/src/
```
To link:
```bash
ln -s path-to-this-folder/starter/beginner_tutorials ~/catkin_ws/src/
```
* Build the package using the `catkin_make` command
* Run a `talker` node using `rosrun beginner_tutorials talker` (Dont forget to start a `roscore` first)
* Run a `listener` node using `rosrun beginner_tutorials listener` 
* Read the source code and try to understand what is going on


Ex03: Modifying a ROS Node
---------------------------

Modify the provided package to do the following:

* Every 0.2 seconds the `talker` node should draw samples from a Gaussian distribution (mean = 0.0, std dev = 1.0) and *publish* the samples to a topic
* The `listener` node should *subscribe* to the topic build a histogram of the numbers received (bin-size = 0.1)
* Upon recieving the 50th number the listener should display the histogram somehow and reset its counts 

### Questions ###

* Do you need to change the Message? refer to http://wiki.ros.org/std_msgs
* Will you use a container for the histogram?
* How would you display it?


[ROS Installation Instructions]: http://wiki.ros.org/ROS/Installation
[ROS Tutorials]: http://wiki.ros.org/ROS/Tutorials
