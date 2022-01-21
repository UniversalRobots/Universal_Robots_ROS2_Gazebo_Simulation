Universal_Robots_ROS2_Gazebo_Simulation
==========================================

Example files and configurations for Gazebo Classic simulation of Universal Robots' manipulators.

## Build status

ROS2 Distro | Branch | Build status | Released packages
:---------: | :----: | :----------: | :---------------:
**Galactic** | [`galactic`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/tree/galactic) | [![Galactic Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/galactic-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/galactic-binary-build.yml?branch=ros2) <br /> [![Galactic Semi-Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/galactic-semi-binary-build.yml?branch=ros2) <br /> [![Galactic Source Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/galactic-source-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/galactic-source-build.yml?branch=ros2) | [ur_simulation_gazebo](https://index.ros.org/p/ur_simulation_gazebo/#galactic)
**Rolling** | [`rolling`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/tree/rolling) | [![Rolling Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-build.yml?branch=ros2) <br /> [![Rolling Semi-Binary Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-semi-binary-build.yml?branch=ros2) <br /> [![Rolling Source Build](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-source-build.yml/badge.svg?branch=ros2)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-source-build.yml?branch=ros2) | [ur_simulation_gazebo](https://index.ros.org/p/ur_simulation_gazebo/#rolling)


### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/Universal_Robots_ROS2_Gazebo_Simulation/Universal_Robots_ROS2_Gazebo_Simulation-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/Universal_Robots_ROS2_Gazebo_Simulation/Universal_Robots_ROS2_Gazebo_Simulation.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


## Using the repository
Skip any of below steps is not applicable.

### Setup ROS Workspace

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspace/ros_ws_foxy
   mkdir -p $COLCON_WS/src
   ```

   > **NOTE:** Feel free to change `~/workspace/ros_ws_foxy` to whatever absolute path you want.

   > **NOTE:** Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ros_ws_foxy`.

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone git@github.com:UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git src/Universal_Robots_ROS2_Gazebo_Simulation
   vcs import src --input src/Universal_Robots_ROS2_Gazebo_Simulation/Universal_Robots_ROS2_Gazebo_Simulation.<ros-distro>.repos
   rosdep install --ignore-src --from-paths src -y -r       # install also is there are unreleased packages
   cd ..
   ```

### Configure and Build Workspace:
To configure and build workspace execute following commands:
  ```
  cd $COLCON_WS
  colcon build --symlink-install --mixin rel-with-deb-info compile-commands ccache
  ```

## Running Simulation
```
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```

Move robot using test script from  `ur_bringup` package:
```
ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
```

Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```
