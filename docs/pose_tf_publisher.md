# pose_tf_publisher

This node do something.

## Parameters

* ```odom_frame``` (default = odom)

The name of the Odom transform that will be published.

* ```child_frame``` (default = base_link)

The name of the robot's root transform.

* ```hz``` (default = 10)

The transform publishing frequency in hertz.

## Subscribed Topics

* ```/pose``` ([geometry_msgs::msg::Pose](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

The pose of the robot to be published as the Odom transform frame.

<!-- ## Published Topics

* ```/share_count``` ([/std_msgs/Int16MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Int16MultiArray.html))

An integer array that shares nearby robots. -->

## Published Transforms

* ```odom```
