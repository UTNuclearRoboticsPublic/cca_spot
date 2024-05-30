#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcAffordancePlannerRos>("cc_affordance_planner_ros", node_options);

    // Start spinning the node in a separate thread so we could do things like reading parameters and joint states
    // inside the node
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });

    /*********************************************************/
    /* All one needs to set or modify to plan, visualize, and execute a task using the Closed-chain Affordance*/
    /* framework is within this code block */

    // Configure the planner
    cc_affordance_planner::PlannerConfig plannerConfig;
    plannerConfig.aff_step = 0.1;
    plannerConfig.accuracy = 10.0 / 100.0;

    // Specify affordance screw info
    affordance_util::ScrewInfo aff;
    aff.type = "translation";
    aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    /* aff.type = "rotation"; */
    /* aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0); */
    /* aff.location = Eigen::Vector3d(1.0, 2.0, 3.0); */
    /* aff.type = "screw"; */
    /* aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0); */
    /* aff.location = Eigen::Vector3d(1.0, 2.0, 3.0); */
    /* aff.pitch = 0.2; */

    // Specify frame name if using apriltag to get affordance locaiton
    /* aff.location_frame = "affordance_frame"; */

    // Specify EE and gripper orientation goals
    const size_t gripper_control_par = 1; // 1 for affordance control only, 2 for affordance plus gripper x orientation,
                                          // 3 for affordance and gripper xy and so on. Set goals accordingly.
    Eigen::VectorXd goal(gripper_control_par);
    const double aff_goal = 1.5 * M_PI;
    goal.tail(1)(0) = aff_goal; // End element
    /* goal[0] = 0.1; // EE orientation goal */
    /*********************************************************/

    // Run the planner
    bool success = node->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par);

    if (success) // If trajectory was successfully executed wait until, goal response callback is invoked and ros is
                 // shutdown from there
    {
        spinner_thread.join(); // join the spinning thread and exit
    }
    else
    {
        rclcpp::shutdown();    // shutdown ROS on failure
        spinner_thread.join(); // join the spinner thread
    }

    return 0;
}
