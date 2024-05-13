# Multi-robot-Intermittent-Rendezvous

This is a package for ROS Noetic that implements an intermittent communication mechanism for multiple robots to accomplish tasks. Soon it is going to be migrated to a ROS 2 version.

# Overview

# License

All content from this repository is released under a [BSD 4-clause license](LICENSE).

Author/Maintainer: Alysson Ribeiro da Silva, multirobotplayground@gmail.com

# Publications

> A. R. da Silva, L. Chaimowicz, T. C. Silva, and A. Hsieh, Communication-Constrained Multi-Robot Exploration with Intermittent Rendezvous. 2023. 

```
@misc{dasilva2023communicationconstrained,
      title={Communication-Constrained Multi-Robot Exploration with Intermittent Rendezvous}, 
      author={Alysson Ribeiro da Silva and Luiz Chaimowicz and Thales Costa Silva and Ani Hsieh},
      year={2023},
      eprint={2309.13494},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

# Commit Guidelines

Write commits with two components:

1. description in imperative mood without ending with a period.
2. sequence of structured paragraphs with detailed explanation of what you have changed.

Example:

```
Remove unused function

The function was unecessary, because of this and that.
Something will be added according to the new roadmap as specified during the meeting.
```

# Building from Source

# Running in Docker

# Usage and Examples

# Launch Files 

* Launch file 1: this launch file do this and that.
  * ```argument 1``` Is used for this and that. Default: ```none```

# Nodes 

## node_1

This node do something.

### Subscribed Topics

* ```/use_network``` ([/std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))

A flag that specifies if the robots must do something.

### Published Topics

* ```/share_count``` ([/std_msgs/Int16MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Int16MultiArray.html))

An integer array that shares nearby robots.

## node_2

This node do something.

### Subscribed Topics

* ```/use_network``` ([/std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))

A flag that specifies if the robots must do something.

### Published Topics

* ```/share_count``` ([/std_msgs/Int16MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Int16MultiArray.html))

An integer array that shares nearby robots.

# Bug & Feature Requests

Please report bugs and do your requests to add new features through the [Issue Tracker](https://github.com/multirobotplayground/Multi-robot-Intermittent-Rendezvous/issues).
