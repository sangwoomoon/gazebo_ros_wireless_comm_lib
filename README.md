# Gazebo/ROS Wireless Comunication Library

This library is for generating `wireless_receiver` topic from ROS/Gazebo environment. `wirless_receiver` creates the `libwireless_receiver.so` as a `gazebo_ros_plugin` module mounted on `catkin_ws/devel/lib`. 

# Contributors

- [Sangwoo Moon](https://bitbucket.org/%7B4b96bc14-1b32-4693-a6e9-575897c81ae4%7D/)

# Workspace Outline

- `include`: header files for `wireless_receiver_ros_plugin.h` which is customized `Gazebo-ROS` plugins.
- `src`: cpp files for `wireless_receiver_ros_plugin.cpp` which is customized `Gazebo-ROS` plugins.

# How to use

You need to insert the communication components (`transmitter.xacro`/`receiver.xacro`) to your `xacro` model file (check `models/cessna/cessna.xacro` for further understanding). Below are example scripts to spawn the transmitter and receiver components respectively:

- Wireless transmitter

        <xacro:include filename="$(find wireless_comm)/models/xacro/transmitter.xacro" />
        <xacro:comm_tx 
            ns="${ns}" 
            update_rate="50" 
            power="18"
            freq="$(arg comm_channel)"
            gain="2.3" 
            parent_link="${ns}/base_link">
        </xacro:comm_tx>

- Wireless receiver

        <xacro:include filename="$(find wireless_comm)/models/xacro/receiver.xacro" />
        <xacro:rf_receiver 
            ns="${ns}" 
            update_rate="50" 
            comm_min_freq="2412" comm_max_freq="5884"
            meas_min_freq="2412" meas_max_freq="5884"
            comm_noise_floor = "-340.0"
            comm_byte_num = "40"
            comm_id_freq = "$(arg comm_channel)"
            gain="2.3" 
            sensitivity="-90.0"
            parent_link="${ns}/base_link">
            <pose>0 0 0 3.141593 0 0</pose>
        </xacro:rf_receiver>

# Communication Model Analysis

- Rx strength in frequency analysis: 500 meter-distance

![](./comm_analysis/histogram_2412Mhz_500.pdf =100x100) ![](./comm_analysis/histogram_2462Mhz_500.pdf =250x250) ![](./comm_analysis/histogram_5180Mhz_500.pdf =250x250) ![](./comm_analysis/histogram_5815Mhz_500.pdf =250x250)

- Rx strength in distance analysis

![](./comm_analysis/histogram_2412Mhz_500.pdf =100x100) ![](./comm_analysis/histogram_2462Mhz_500.pdf =250x250) ![](./comm_analysis/histogram_5180Mhz_500.pdf =250x250) ![](./comm_analysis/histogram_5815Mhz_500.pdf =250x250)

- Rx strength analysis compared with real data

![Signal strength in distance](./comm_analysis/boxplot_range.pdf =250x250) ![Signal strength in distance compared with real communication data](./comm_analysis/beta_profile.pdf =250x250)

# Reference

S. Moon, J. J. Bird, S. Borenstein and E. W. Frew, "A Gazebo/ROS-based Communication-Realistic Simulator for Networked sUAS," 2020 International Conference on Unmanned Aircraft Systems (ICUAS), 2020, pp. 1819-1827, doi: 10.1109/ICUAS48674.2020.9213892. [link](https://ieeexplore.ieee.org/abstract/document/9213892)
