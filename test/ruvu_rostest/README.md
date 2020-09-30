<!--
Copyright 2020 RUVU Robotics B.V.
-->

# RUVU rostest

Generic test using [rostest](http://wiki.ros.org/rostest).

## test_twist_odom_has_movement.py

Publishes a fixed velocity in `x` amd `y` over the `cmd_vel` topic for a timeout in seconds defined by the `~timeout` parameter. After this period, the position, received over the `odom` topic is evaluated. The test is succeeded if an odom message is received and the position is not equal to zero.