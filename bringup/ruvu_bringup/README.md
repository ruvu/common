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