<!--
Copyright 2020 RUVU Robotics B.V.
-->

# RUVU Bringup

Scripts for bringing up applications / drivers / robots

## generate_functions_for_launch_files_in_package.bash

Script for generating bash functions for launch files in packages. Usage:

	source generate_functions_for_launch_files_in_package.bash [package_name] [function_prefix]

So if we have the following bringup_package, e.g. myawesomerobotname_simulation_bringup with the following structure:

```
myawesomerobotname_simulation_bringup
myawesomerobotname_simulation_bringup/launch
myawesomerobotname_simulation_bringup/launch/robot_interfaces.launch
myawesomerobotname_simulation_bringup/launch/test.launch
```
You can let the script generate bash functions for these launch files:

	source generate_functions_for_launch_files_in_package.bash myawesomerobotname_simulation_bringup myawesomerobotname

The result will be

```
myawesomerobotname-robot-interfaces () --> roslaunch myawesomerobotname_simulation_bringup robot_interfaces.launch
myawesomerobotname-test () --> roslaunch myawesomerobotname_simulation_bringup test.launch

```

## set_ros_ip_from_network_interface.bash

Script for setting the ROS_IP based on a network interface. Usage:

	source set_ros_ip_from_network_interface.bash network_interface, e.g. eth0, eno1, wlan1]

This will then export the ROS_IP to the found IP address, or print and error.

## systemd_service_from_launch_file.py

```
usage: systemd_service_from_launch_file.py [-h]
                                           launch_file user environment_bash

systemd_service_from_launch_file

positional arguments:
  launch_file       Launch file to generate a service from
  user              The user that should execute the launch file
  environment_bash  Bash environment to run before

optional arguments:
  -h, --help        show this help message and exit
```
