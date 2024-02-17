# Closed-chain Affordance Planning for the NRG Spot Robot using ROS2

## Build Instructions:
1. Clone this repository onto your local machine ROS2 workspace `src` folder:
   ```
   cd ~/<ros2_ws_name>/src
   ```
   ```
   git clone -b main git@github.com:UTNuclearRobotics/closed_chain_affordance_spot.git
   ```

2. Build and source the `cca_spot` package:
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

### On the Spot Core:

1. Run the Spot driver:
   ```
   ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3
   ```

2. Launch the robot state publisher:
   ```
   ros2 launch spot_description state_publisher.launch.py has_arm:=True
   ```

### On Your Local Machine:

3. Load `robot_description_semantic` onto the parameter server. For convenience, we'll do it by launching the Moveit motion planning plugin file, which not only publishes `robot_description_semantic`, but also opens up `Rviz`.
   ```
   ros2 launch spot_arm_moveit2_config spot_arm_planning_execution.launch.py joint_state_topic:=/spot_driver/joint_states
   ```

4. Run the planning visualization server. Then, in RViz, add MarkerArray with publishing topic, `/ee_trajectory`.
   ```
   ros2 run moveit_plan_and_viz moveit_plan_and_viz_node
   ```

5. Load closed-chain affordance description for Spot and run the closed-chain affordance planner node:
   ```
   ros2 launch cca_spot cca_spot_launch.py
   ```
