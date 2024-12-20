<!--
Copyright 2020 RUVU Robotics B.V.
-->

# RUVU Navigation playground

## Research

Planner / Controller combinations can be tested using:

````
roslaunch ruvu_navigation_playground default_in_empty_world.test
````

### Planners

| Planner name | Notes |
|-----------------|-------|
| [carrot_planner/CarrotPlanner](http://wiki.ros.org/carrot_planner) | Creates path of size 2 (not handled properly by a lot of controllers) |
| [goal_passer/GoalPasser](https://github.com/ros-planning/navigation_experimental/tree/hydro-devel/goal_passer) | Creates path of size 1, not handled properly by most controllers |
| [navfn/NavfnROS](http://wiki.ros.org/navfn) | A* search |
| [global_planner/GlobalPlanner](http://wiki.ros.org/global_planner) | Replacement of navfn |

### Controllers

| Controller name | Notes |
|-----------------|-------|
| [dwa_local_planner/DWAPlannerROS](http://wiki.ros.org/dwa_local_planner) | Hard to configure for line following |
| [simple_local_planner/PurePlannerROS (simple_local_planner)](https://github.com/robotics-upo/upo_robot_navigation/tree/master/simple_local_planner) | Works out of the box but not with sparse paths. Does not work yet on melodic. |
| [ftc_local_planner/FTCPlanner](http://wiki.ros.org/asr_ftc_local_planner)| Does not work out of the box, no proper nav_core interface, bad code |
| [eband_local_planner/EBandPlannerROS](http://wiki.ros.org/eband_local_planner) | A lot of parameters, most probably too complex |
| [pose_follower/PoseFollower](https://github.com/ros-planning/navigation_experimental/tree/hydro-devel/pose_follower) | Sends NaN sometimes |
