# GRAIC

GRAIC autonomous racing competition

1. Place this ROS package in the src folder of the catkin workspace. Name the folder of the package as 'GoHeelsRacing'.
2. Download and unzip the C++ Boost 1.72.0 library to ~/Downloads/boost_1_72_0 (or adjust the 'include_directories' in CMakeLists.txt).
3. Run catkin_make to build the package.
4. Run 'roslaunch race carla_single.launch' for the race system.
5. Run 'roslaunch GoHeelsRacing race.launch' for our controller.
