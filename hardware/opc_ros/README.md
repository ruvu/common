<!--
Copyright 2020 RUVU Robotics B.V.
-->

# OPC ROS

[Open Pixel Client](http://openpixelcontrol.org/) for ROS.

## Node

Connects to a running OPC Server on the localization.

```
rosrun opc_ros opc_ros
```

### Test scripts

#### Send random colors

```
rosrun opc_ros send_random_colors 
```

```
usage: send_random_colors [-h] [--number_of_pixels NUMBER_OF_PIXELS]

Send colors via ROS to a led array

optional arguments:
  -h, --help            show this help message and exit
  --number_of_pixels NUMBER_OF_PIXELS
                        Number of pixels
```

#### Send color

```
rosrun opc_ros send_color 
```

```
usage: send_color [-h] [--number_of_pixels NUMBER_OF_PIXELS] RED GREEN BLUE

Send colors via ROS to a led array

positional arguments:
  RED                   Red value
  GREEN                 Green value
  BLUE                  Blue value

optional arguments:
  -h, --help            show this help message and exit
  --number_of_pixels NUMBER_OF_PIXELS
                        Number of pixels
```