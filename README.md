# WIRELESS_RECEIVER

This library is for generating `wireless_receiver` topic from ROS/Gazebo environment. `wirless_receiver` creates the `libwireless_receiver.so` as a `gazebo_ros_plugin` module mounted on `catkin_ws/devel/lib`. The module is used for `wireless transciver` module in `rf_receiver.xacro` model of `mass_cu` package.

# Contributors

- [Sangwoo Moon](https://bitbucket.org/%7B4b96bc14-1b32-4693-a6e9-575897c81ae4%7D/)

# Workspace Outline

**DUE TO THE ISSUE OF AIRCRAFT SPAWNING, THE SENSOR ALSO RECEIVES SIGNALS FROM AIRCRAFT**
- `include`: header files for `wireless_receiver_ros_plugin.h` which is customized `Gazebo-ROS` plugins.
- `src`: cpp files for `wireless_receiver_ros_plugin.cpp` which is customized `Gazebo-ROS` plugins.
