# [Table of Contents](#table-of-contents)

- [Motivation](docs/motivation.md)
- [Packages](#packages)
- [Setup](docs/working_environment.md)
- [Robots](docs/robots.md)
- [Contributing](docs/contributing.md)

## [Multi-robot-Sandbox](#multi-robot-sandbox)

This package for ROS 2 Jazzy Jalisco and Ubuntu 24.04 has a deployment that allows you to work with multiple robots in Ignition Gazebo. It provides you an environment for heterogeneous robots, UAVS and UGVS, that publishes the correct transformation trees and topics to controll mobile robots out-of-the-box. It might help you to further your research or development faster without prior knowledge necessary to configure simulations within the ROS 2 environment, such as operational systems, computer networks, parallel computing, simulations architectures, linear algebra, and so on.

## [Packages](#packages)

For now, this repositoy contains the following packages and nodes.

1. [multi-robot-simulations](#multi-robot-sandbox)
   - [PoseTFPublisher node](docs/pose_tf_publisher.md)

It contains simulations that allows you to start your development out-of-the-box. There is a main launch file which already take care of everything for you and simply bring-up all robots, transforms, and topics organized by robot namespace.

## [Publications](#publications)

If this package is somehow useful to you, consider reading this [letter](movitation.md) and mentioning this ongoing research.

> A. R. da Silva, L. Chaimowicz, T. C. Silva, and A. Hsieh, Communication-Constrained Multi-Robot Exploration with Intermittent Rendezvous. 2023.

```text
@misc{dasilva2023communicationconstrained,
      title={Communication-Constrained Multi-Robot Exploration with Intermittent Rendezvous}, 
      author={Alysson Ribeiro da Silva and Luiz Chaimowicz and Thales Costa Silva and Ani Hsieh},
      year={2023},
      eprint={2309.13494},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## [License](#license)

All content from this repository is released under a [BSD 4-clause license](LICENSE).

Author/Maintainer: Alysson Ribeiro da Silva, <multirobotplayground@gmail.com>

## [Bug & Feature Requests](#bug--feature-requests)

Please report bugs and do your requests to add new features through the [Issue Tracker](https://github.com/multirobotplayground/Multi-robot-Intermittent-Rendezvous/issues).
