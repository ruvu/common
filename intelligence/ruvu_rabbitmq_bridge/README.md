# RUVU RabbitMQ Bridge

Bridges between [ROS](http://ros.org) and [RabbitMQ](https://www.rabbitmq.com/).

## Nodes

### rabbitmq_bridge

```
rosrun ruvu_rabbitmq_bridge rabbitmq_bridge
```

In and output topic are specified using the parameters.

#### Parameters

- `~subscribers`: List with topic and message type entries. A ROS subscriber will be created and every message will be published to RabbitMQ.
- `~publishers`: List with topic and message type entries. A RabbitMQ subscriber will be created and every message will be published to the ROS network on the specified topic.
- `~rabbitmq_queue_prefix`: This prefix will be prepended to the specified topic names in the `~subscribers` and `~publishers` parameters for the RabbitMQ queue name.
- `~ros_publishere_queue_size`: Queue size on the ROS end (only holds for the specified `~publishers`).

Example parameter file:

```
subscribers:
  - topic: string_to_rabbitmq
    message_type: std_msgs/String
  - topic: float_to_rabbitmq
    message_type: std_msgs/Float32
publishers:
  - topic: string_from_rabbitmq
    message_type: std_msgs/String
  - topic: twist_from_rabbitmq
    message_type: geometry_msgs/Twist
rabbitmq_host: localhost
#rabbitmq_queue_prefix: custom_prefix # defaults to current hostname
ros_publisher_queue_size: 10 # defaults to 10
```
