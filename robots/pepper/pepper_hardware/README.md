# pepper_hardware

[![Build Status](https://travis-ci.org/ruvu/pepper_hardware.png)](https://gitlab.com/ruvu/robot/pepper/pepper_hardware/pipelines/12040628)

Hardware interfacing for the Pepper robot

## Setting up

- Clone this repo in your catkin workspace,
- build your workspace,
- source your environment.

## Running
 - Make sure you're on the same network as the Pepper you're connecting to,
 - find out the IP or hostname of the robot,
 - launch the pepper base proxy:
 ```
 roslaunch pepper_hardware full.launch robot_ip:=<ip.or.hostname>
 ```

## Usage
You should now have a ROS interface for the pepper, meaning you should be
able to send std_msgs/Twist messages to a topic called cmd_vel, for example.
Test this with a teleop node, and make sure you run it in the right namespace.

## Tips and tricks
Make sure you disable Autonomous Life to make sure your ROS layer has full
control over the robot.
