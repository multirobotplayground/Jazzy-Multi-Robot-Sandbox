# pose_tf_publisher

This node gets the pose of a robot and creates a Odom frame as a static transform. It publishes it as a static transform so RViz can look for the correct transform and visualize the correct point clounds and related sensor messages that depends on a global reference frame. For some reason, when I publish this as a dynamic transform, it is unable to find it in the transformation tree.

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
