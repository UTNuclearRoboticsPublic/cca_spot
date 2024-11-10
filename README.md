# Closed-Chain Affordance Planning for NRG Spot using ROS2
This repository contains the `cca_spot` package, which implements the closed-chain affordance planning framework on the manipulator-integrated Boston Dynamics Spot robot.

## Build and Install Instructions:
1. Install the closed-chain affordance planning libraries by following instructions from the following repository:
   [Link to instructions](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_ros.git)

2. Clone this repository onto your machine ROS2 workspace `src` folder:
   ```
   cd ~/<ros2_ws_name>/src
   ```
   ```
   git clone -b main git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_spot.git
   ```

3. Build and source the `cca_spot` package:
   ```
   cd ~/<ros2_ws_name>
   ```
   ```
   colcon build --packages-select cca_spot
   ```
   ```
   source install/setup.bash
   ```

## Run Instructions:

### Using With a Physical Robot
To execute CCA-generated trajectories on the NRG Spot robot:

#### On the Spot Core

1. Run the Spot driver:
   ```
   ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3
   ```

#### On Your Local Machine

2. Launch the CCA visualizer for Spot:

   ```
   ros2 launch cca_spot cca_spot_viz.launch.py
   ```

3. Launch the CCA planner/executor:
   ```
   ros2 launch cca_spot cca_spot.launch.py
   ```
### Using Without a Physical Robot
You can plan and visualize trajectories for the BD Spot using the CCA framework without needing a physical robot. The following demonstration showcases various CCA framework features on Spot.

1. Launch the CCA-visualizer for Spot:

   ```
   ros2 launch cca_spot cca_spot_viz.launch.py
   ```

2. Launch the CCA planner demo node:
   ```
   ros2 launch cca_spot cca_spot_demo.launch.py
   ```

You are encouraged to modify the tasks in the demo node to plan and visualize trajectories tailored to your specific applications. Task examples are also provided in the package README.md.

## Author
Janak Panthi aka Crasun Jans
