/*************************************/
// Author: Crasun Jans
// Description:
// This node enables users to plan, visualize, and execute robot joint trajectories for specified tasks. The planning
// process utilizes the Closed-chain Affordance model, as described in the paper:
// "A closed-chain approach to generating affordance joint trajectories for robotic manipulators."
//
// Usage Instructions:
// 1. The framework requires only two inputs: planner configuration and task description. See the examples provided
//    in the code for various affordance and motion types and how to define these inputs.
// 2. The main function has comment blocks for the following use cases. Uncomment as needed.
//    - Basic Use Case: Plan, visualize, and execute a joint trajectory from the current robot configuration.
//    - Optional Advanced Use Cases:
//    - Optional Use Case 1: Plan, visualize, and execute while tracking the status of planning and execution.
//    - Optional Use Case 2: Plan, visualize, and execute from a desired robot start configuration, or plan and
//      visualize without connecting to a real robot.
// Important Note:
// You can run the example tasks directly on the robot. However, in the basic use case, the framework plans
// from the robot's current configuration, which may not be suitable for these examples. As a result, we
// recommend using Optional Use Case 2 to run the example tasks and visualize them in RVIZ. Case 2 provides
// a predefined start configuration to ensure compatibility with the examples.
/*************************************/
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

enum ExampleType
{
    AFFORDANCE_TRANSLATION,
    AFFORDANCE_ROTATION,
    AFFORDANCE_SCREW,
    APPROACH,
};

// Function to generate example planner configurations and task descriptions based on the ExampleType
std::pair<cc_affordance_planner::PlannerConfig, cc_affordance_planner::TaskDescription>
get_example_planner_config_and_task_description(const ExampleType &example_type)
{
    cc_affordance_planner::PlannerConfig planner_config;
    planner_config.accuracy = 10.0 / 100.0;
    planner_config.trajectory_density = 10;

    affordance_util::ScrewInfo aff;
    Eigen::VectorXd aff_goal;
    cc_affordance_planner::TaskDescription task_description;

    switch (example_type)
    {
    case ExampleType::AFFORDANCE_ROTATION:
        aff.type = affordance_util::ScrewType::ROTATION;
        aff.axis = Eigen::Vector3d(0, 0, 1);
        aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
        aff_goal = (Eigen::VectorXd(1) << (1.0 / 2.0) * M_PI).finished();
        task_description.affordance_info = aff;
        task_description.nof_secondary_joints = 1;
        task_description.secondary_joint_goals = aff_goal;
        break;

    case ExampleType::AFFORDANCE_TRANSLATION:
        aff.type = affordance_util::ScrewType::TRANSLATION;
        aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0);
        aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
        aff_goal = (Eigen::VectorXd(1) << 0.24).finished();
        task_description.affordance_info = aff;
        task_description.nof_secondary_joints = 1;
        task_description.secondary_joint_goals = aff_goal;
        break;

    case ExampleType::AFFORDANCE_SCREW:
        aff.type = affordance_util::ScrewType::SCREW;
        aff.axis = Eigen::Vector3d(0, 0, 1);
        aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
        aff.pitch = 0.5;
        aff_goal = (Eigen::VectorXd(1) << (1.0 / 2.0) * M_PI).finished();
        task_description.affordance_info = aff;
        task_description.nof_secondary_joints = 1;
        task_description.secondary_joint_goals = aff_goal;
        break;

    case ExampleType::APPROACH:
        aff.type = affordance_util::ScrewType::ROTATION;
        aff.axis = Eigen::Vector3d(0, 0, 1);
        aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
        aff_goal = (Eigen::VectorXd(1) << 0.0).finished();

        Eigen::Matrix4d approach_pose;
        approach_pose << 0.998453, 0.0378028, 0.0407843, 0.529228, -0.0380367, 0.999264, 0.00497515, -0.16148,
            -0.0405662, -0.00651876, 0.999156, 0.100135, 0, 0, 0, 1;

        task_description.motion_type = cc_affordance_planner::MotionType::APPROACH;
        task_description.affordance_info = aff;
        task_description.nof_secondary_joints = 2;
        task_description.secondary_joint_goals = (Eigen::VectorXd(2) << 0, aff_goal).finished();
        task_description.grasp_pose = approach_pose;
        break;
    }

    return std::make_pair(planner_config, task_description);
}

// Function to block until the robot completes the planned trajectory
void block_until_trajectory_execution(const std::shared_ptr<cc_affordance_planner_ros::Status> &motion_status,
                                      const rclcpp::Logger &logger)
{
    rclcpp::Rate loop_rate(4);
    while (*motion_status != cc_affordance_planner_ros::Status::SUCCEEDED)
    {
        if (*motion_status == cc_affordance_planner_ros::Status::UNKNOWN)
        {
            RCLCPP_ERROR(logger, "Motion was interrupted mid-execution.");
        }
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node =
        std::make_shared<cc_affordance_planner_ros::CcAffordancePlannerRos>("cc_affordance_planner_ros", node_options);

    // Start spinning the node in a separate thread to enable ROS functionalities like parameter reading and joint
    // states
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });

    rclcpp::sleep_for(std::chrono::seconds(1)); // Sleep for 1 second to ensure ROS is initialized properly

    /// REQUIRED INPUT: Planner configuration and task description. Example provided below.
    auto [planner_config, task_description] =
        get_example_planner_config_and_task_description(ExampleType::AFFORDANCE_ROTATION);

    /*******************************************/
    // BASIC USE CASE: Plan and execute a joint trajectory for a given task from the current robot configuration
    /* if (!(node->run_cc_affordance_planner(planner_config, task_description))) */
    /* { */
    /*     RCLCPP_ERROR(node->get_logger(), "Planning and execution failed"); */
    /* } */

    /*******************************************/
    // OPTIONAL USE CASE 1: Plan, visualize, and execute while tracking the planning and execution status
    /* auto motion_status = */
    /*     std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN); */
    /* if (!(node->run_cc_affordance_planner(planner_config, task_description, motion_status))) */
    /* { */
    /*     RCLCPP_ERROR(node->get_logger(), "Planning and execution failed"); */
    /*     rclcpp::shutdown(); */
    /*     return -1; */
    /* } */
    /* block_until_trajectory_execution(motion_status, node->get_logger()); */

    /*******************************************/
    // OPTIONAL USE CASE 2: Plan, visualize, and execute from a desired start robot configuration
    const Eigen::VectorXd robot_start_config =
        (Eigen::VectorXd(6) << 0.0, -1.09419, 2.2496, -0.567882, -0.796551,
         0.396139)
            .finished(); // Example robot configuration for planning and visualization
    auto motion_status =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);

    if (!(node->run_cc_affordance_planner(planner_config, task_description, motion_status, robot_start_config)))
    {
        RCLCPP_ERROR(node->get_logger(), "Planning and execution failed");
        rclcpp::shutdown();
        return -1;
    }

    block_until_trajectory_execution(motion_status, node->get_logger());

    rclcpp::shutdown();
    return 0;
}
