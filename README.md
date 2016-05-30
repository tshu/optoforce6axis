# optoforce6axis
A custom C++ API to communicate with OptoForce's 6-axis transducers, compatible with both 32 and 64bit Linux systems. Written for v1.4 of their USB DAQs. 

http://optoforce.com/sixaxis/

To run:

1. Make sure you have ROS installed.
2. Create a catkin_ws and clone the repo into the empty workspace.
3. ```catkin_make``` in the workspace root.
4. ```source devel/setup.bash```
5. In a new terminal, ```roscore``
6. ```rosrun optoforce optoforce_node``` with transducer connected.

alternatively,

1. Use your favorite toolchain with the optoDriver.hpp, optoDriver.cpp, and test.cpp files.


