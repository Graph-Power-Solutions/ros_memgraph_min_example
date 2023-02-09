## Mininal example of ROS2 and MemGraph interaction

Mininal example for [MemGraph](https://www.memgraph.com/) for testing on real [ROS2](https://ros.org) devices.

- Using: [mgclient](https://github.com/memgraph/mgclient) - C++ implementation of [Bolt](https://boltprotocol.org/) connector for graph databases.

## Installation
1. [Install ROS2](https://docs.ros.org/en/humble/Installation.html)
2. Activate your workplace (e.g. with `source /opt/ros/humble/setup.bash`).
3. In your ROS2 workplace directory:
```
cd src
git clone https://github.com/Graph-Power-Solutions/ros_memgraph_min_example
cd ..
colcon build --symlink-install
. install/setup.bash
ros2 run ros_memgraph_min_example query
```

## More information
- https://graph-power.com