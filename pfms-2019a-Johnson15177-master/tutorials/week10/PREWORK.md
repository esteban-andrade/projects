Week 10 Pre-work
=========================

Work through these questions and make sure you understand what is going on in each example. 
If you have any questions about the material please raise them in the next interactive leacture / tutorial session.

Now we will take a ROS Node and modify its code to make it do something new. This should help you get familiar with writing software for ROS.

The pre-work and tutorial requires the a5_support packages to be part of your ros system

* The a5_setup packages are provided in the [a5_setup](../../skeleton/a5_setup) directory. To use it or link it to your workspace.
To link:
```bash
ln -s path-to-your-repository/skeleton/a5_setups ~/catkin_ws/src/
```
* Build the package using the `catkin_make` command
* Start the roscore using the `roscore` command
launch the a5_setup packages:
```bash
roslaunch a5_setup a5_setup.launch
```

* A boiler plate package is provided in the [starter](./starter) directory. To use it either copy or link it to your workspace.
To link:
```bash
ln -s path-to-this-folder/starter/topics_masterclass ~/catkin_ws/src/
```
* Build the package using the `catkin_make` command
* Read the source code and try to understand what is going on

Modify the provided package to do the following excersize (Ex01), we continue with excersies in tutorial.

Ex01: Obtain pose of robot (x,y yaw) from nav_msgs/Odometry
-----------------------------------------------

Find the code for Ex01

* On command line type 'rosmsg show nav_msgs/Odometry'
* Identify if position and orientation are in two seperate parts of the message
* The orinetation is provided as a quaternion
* Ros has a [ROS TF] library with a helper function to get yaw from the quaternion

Use all the information above to obtain the pose of robot (x,y yaw) from nav_msgs/Odometry

### Questions ###

* Do we have nav_msgs::Odometry or q pointer to nav_msgs::Odometry ?
* Where is time of this message stored
* Which angle to we need?

[ROS TF]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
