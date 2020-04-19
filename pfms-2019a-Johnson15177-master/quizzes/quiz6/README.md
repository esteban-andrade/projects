Quiz 6
======

INFORMATION
------
- To develop the quiz 

symbolically link the [quiz6 folder](.) to your catkin_ws/src (ie if your quiz6 folder path is /home/student/git/pfms-2019a-<YOURHANDLE>/quizzes/quiz6/ then execute:
```bash
cd ~/catkin_ws/src
ln -s /home/student/git/pfms-2019a-<YOURHANDLE>/quizzes/quiz6/
```
- Compile with catkin_make
```bash
cd ~/catkin_ws
catkin_make
```

- You will need to run the a5_setup to enable testing your code
```bash
roslaunch a5_setup a5_setup.launch
```
- When running your package for Quiz6a
```bash
rosrun quiz6a quiz6a-sample
```
- When running your package for Quiz6b (substitute XXX for name of node)
```bash
rosrun quiz6b quiz6a-XXX
```
- Sections for TASKS in the quiz have a `(//! @todo)` in code to enable identification

Part A
------
1) QUESTION: Look at the code in [folder a](./a), what is the name of the package, and node(s)?
ANSWER: quiz_6a, the node is nh

2) TASK: In the code [seperateThread](./a/src/sample.cpp) "ros::Rate rate_limiter(XXX)" located in [sample.cpp](./a/sample.cpp) achieve running thread at specific rate. Adjust it to run every 5 seconds.

3) QUESTION: In the code [requestGoal](./a/src/sample.cpp) we utilise a RequestGoal service. In what facility is it: (A) offer a service (recieve request) or (B) client (makes request).  
ANSWER: A

4) TASK: In the code [requestGoal](./a/src/sample.cpp) augument the RequestGoal service to return true if the global coordinate supplied in the requst is within current OgMap (just needs to be within the map - either free / occupied or unknown).

After running `rosrun quiz6a quiz6a-sample`
You wil be able to request a goal via the terminal, type below
```bash
rosservice call /check_goal
```
then hit *TAB* twice, will auto populate with entier message, edit x,y and hit enter to test code.
```bash
rosservice call /check_goal "x: 0.0
y: 0.0
theta: 0.0"
```

5) TASK: In the code [seperateThread](./a/src/sample.cpp), draw a circle at global location P(x,y)=(8.0,8.0) on the OgMap


Part B
------
1) QUESTION: Look at the code in [folder b](./b), what is the name of the package(s) and node(s)? 
ANSWER: the package is quiz_6b, the nodes is nh_, nh, pn

2) TASK: In [Consutsrctur of PfmsSample](./b/src/plot_path.cpp) create a client for a5_help::RequestGoal on "check_goal"

3) TASK: In [pathCallback of PfmsSample](./b/src/plot_path.cpp) add the series of poses received onto the pathArrayBuffer_

4) TASK: In [seperateThread of PfmsSample](./b/src/plot_path.cpp), draw the poses on the path (positions) onto the OgMap

5) TASK: In [seperateThread of PfmsSample](./b/src/plot_path.cpp), call the client for a5_help::RequestGoal with the goal the last (final) pose of the path

To test task 2-5 run the quiz6b-gen_path node `rosrun quiz6b quiz6b-gen_path`

Left click as many time as you wish on the image (these are poses to be sent across to your quiz6b-plot_path node). When you right click the poses are sent.
