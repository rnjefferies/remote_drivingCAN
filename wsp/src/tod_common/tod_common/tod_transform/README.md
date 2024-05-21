# tod_transform

Contains tf-tree broadcasters for odom and static vehicle tree. For the information on dependencies please refer to the `package.xml`.

## Nodes

The package consists of the following set of nodes for both, the vehicle and operator side.

### CommonOdomTransformPublisher

**Subscription**: `/*/VehicleBridge/odometry` ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)) `*`: Vehicle/Operator

### CommonTransformTreePublisher

**Publication**: `/tf_static` [(tf2_msgs/TFMessage)](http://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html)
