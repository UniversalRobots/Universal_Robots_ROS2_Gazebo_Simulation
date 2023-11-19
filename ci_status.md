## Build Status

This page gives a detailed overview of the build status of this repository. Please note that due to
upstream changes some pipelines might turn red temporarily which can be expected behavior.

<table width="100%">
  <tr>
    <th></th>
    <th>Humble</th>
    <th>Iron</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2">ros2</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2">ros2</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/ros2">ros2</a></td>
  </tr>
  <tr>
    <th>Repo builds</th>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-binary-main.yml/badge.svg?event=schedule"
              alt="Humble Binary Main"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-binary-testing.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-binary-testing.yml/badge.svg?event=schedule"
              alt="Humble Binary Testing"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-semi-binary-main.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-semi-binary-main.yml/badge.svg?branch=main"
              alt="Humble Semi-Binary Main"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-semi-binary-testing.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/humble-semi-binary-testing.yml/badge.svg?branch=main"
              alt="Humble Semi-Binary Testing"/>
      </a>
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-binary-main.yml/badge.svg?event=schedule"
              alt="Iron Binary Main"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-binary-testing.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-binary-testing.yml/badge.svg?event=schedule"
              alt="Iron Binary Testing"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-semi-binary-main.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-semi-binary-main.yml/badge.svg?branch=main"
              alt="Iron Semi-Binary Main"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-semi-binary-testing.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/iron-semi-binary-testing.yml/badge.svg?branch=main"
              alt="Iron Semi-Binary Testing"/>
      </a>
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-main.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-main.yml/badge.svg?event=schedule"
              alt="Rolling Binary Main"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-testing.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-binary-testing.yml/badge.svg?event=schedule"
              alt="Rolling Binary Testing"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-semi-binary-main.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-semi-binary-main.yml/badge.svg?branch=main"
              alt="Rolling Semi-Binary Main"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-semi-binary-testing.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation/actions/workflows/rolling-semi-binary-testing.yml/badge.svg?branch=main"
              alt="Rolling Semi-Binary Testing"/>
      </a>
    </td>
  </tr>
</table>

## Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that
   direct local build is possible and integration tests work against the released state.

   Uses repos file: `src/Universal_Robots_ROS2_Gazebo_Simulation/Universal_Robots_ROS2_Gazebo_Simulation-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if this fails we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/Universal_Robots_ROS2_Gazebo_Simulation/Universal_Robots_ROS2_Gazebo_Simulation.repos`
