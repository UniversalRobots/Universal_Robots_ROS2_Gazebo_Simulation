Universal_Robots_ROS2_Gazebo_Simulation
==========================================

Example files and configurations for Gazebo Classic simulation of Universal Robots' manipulators.

## Build status
<table width="100%">
  <tr>
    <th></th>
    <th>Humble</th>
    <th>Iron</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble">humble</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2">ros2</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2">ros2</a></td>
  </tr>
  <tr>
    <th>Build status</th>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-binary-main.yml/badge.svg?event=schedule"
              alt="Humble Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-binary-main.yml/badge.svg?event=schedule"
              alt="Iron Binary Main"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-main.yml/badge.svg?event=schedule"
              alt="Rolling Binary Main"/>
      </a> <br />
    </td>
  </tr>
</table>

A more [detailed build status](ci_status.md) shows the state of all CI workflows inside this repo.
Please note that the detailed view is intended for developers, while the one here should give end
users an overview of the current released state.

## Using the repository
Skip any of below steps is not applicable.

### Setup ROS Workspace

1. Create a colcon workspace:
   ```
   export COLCON_WS=~/workspaces/ur_gazebo
   mkdir -p $COLCON_WS/src
   ```

   > **NOTE:** Feel free to change `~/workspaces/ur_gazebo` to whatever absolute path you want.

   > **NOTE:** Over time you will probably have multiple ROS workspaces, so it makes sense to them all in a subfolder.
     Also, it is good practice to put the ROS version in the name of the workspace, for different tests you could just add a suffix to the base name `ur_gazebo`.

1. Download the required repositories and install package dependencies:
   ```
   cd $COLCON_WS
   git clone git@github.com:UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git src/Universal_Robots_ROS2_Gazebo_Simulation
   vcs import src --input src/Universal_Robots_ROS2_Gazebo_Simulation/Universal_Robots_ROS2_Gazebo_Simulation.<ros-distro>.repos
   rosdep install --ignore-src --from-paths src -y
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

Move robot using test script from  `ur_robot_driver` package (if you've installed that one):
```
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```
