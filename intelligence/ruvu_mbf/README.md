# RUVU MBF

Move base flex tools / scripts.

## get_path_exe_path_relay

Relays from a generic MBF navigation server to specific implementations.

### Parameters

- controllers
- planners

Example:

```
# Exposes the navigation actionlib interfaces and relays these to the various navigation implementations

controllers:
    - name: cart_docking_controller
      actionlib: cart_docking/exe_path
      controller_name: cart_docking_controller
    - name: ftc_controller
      actionlib: costmap/exe_path
      controller_name: ftc_controller
    - name: x_y_yaw_controller
      actionlib: costmap/exe_path
      controller_name: x_y_yaw_controller
    - name: y_x_yaw_controller
      actionlib: costmap/exe_path
      controller_name: y_x_yaw_controller
    - name: yaw_y_x_controller
      actionlib: costmap/exe_path
      controller_name: yaw_y_x_controller

planners:
    - name: goal_passer
      actionlib: costmap/get_path
      planner_name: goal_passer
    - name: forward_grid_planner
      actionlib: costmap/get_path
      planner_name: forward_grid_planner
    - name: backward_grid_planner
      actionlib: costmap/get_path
      planner_name: backward_grid_planner
    - name: leftward_grid_planner
      actionlib: costmap/get_path
      planner_name: leftward_grid_planner
    - name: rightward_grid_planner
      actionlib: costmap/get_path
      planner_name: rightward_grid_planner
```

## pose_stamped_move_base_goal

Exposes `geometry_msgs/PoseStamped` interfaces (can be send by RVIZ) for every planner / controller combination.

### Parameters

- controllers
- planners
