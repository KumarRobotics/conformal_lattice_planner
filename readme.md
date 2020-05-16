# Conformal Lattice Planner

This respository provides conformal lattice planner C++ implementations for autonomous driving tasks. The software is build upon [Carla](http://carla.org/) and [ROS](https://www.ros.org/).

## Related Publications

* K. Sun, B. Schlotfelt, S. Chaves, P. Martin, G. Mandhyan, and V. Kumar, _"Feedback Enhanced Motion Planning for Autonomous Vehicles"_, Robotics and Automation Letter, 2020 (under revision).

<iframe width="560" height="315" src="https://www.youtube.com/embed/TxMY1dvHFog" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## License

BSD 3-clause License

## Workspace Setup

The setup of the workspace can be a bit tricky, mostly because Carla 0.9.6 uses Boost 1.69 but Ubuntu 18.04 has Boost 1.65 by default. Therefore, we have to complie almost all of the dependencies from source and link against the customized compiled libraries. All dependencies are listed as follows. The details of the setup can be found [here](scripts/workspace_setup.md).

### Dependencies
* [Boost 1.69.0](https://www.boost.org/doc/libs/1_69_0/)
* [PCL 1.9.1](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.9.1)
* [ROS Melodic](http://wiki.ros.org/melodic)
* [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
* [Carla 0.9.6](https://carla.org/2019/07/12/release-0.9.6/)

## Software Framework

