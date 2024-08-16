#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

int main(int argc, char **argv)
{
    /*rclcpp::init(argc, argv); */
    /*rclcpp::NodeOptions node_options; */
    /*node_options.automatically_declare_parameters_from_overrides(true); */
    /*auto node = */
    /*    std::make_shared<cc_affordance_planner_ros::CcAffordancePlannerRos>("cc_affordance_planner_ros",
     * node_options); */

    /*// Start spinning the node in a separate thread so we could do things like reading parameters and joint states */
    /*// inside the node */
    /*std::thread spinner_thread([&node]() { rclcpp::spin(node); }); */

    /*rclcpp::sleep_for(std::chrono::seconds(1)); // Sleep for 1s to ensure affordance start joint state is read */
    /*/1*********************************************************/
    /*/1* All one needs to set or modify to plan, visualize, and execute a task using the Closed-chain Affordance*/
    /*/1* framework is within this code block *1/ */

    /*// Configure the planner */
    /*cc_affordance_planner::PlannerConfig plannerConfig; */
    /*plannerConfig.aff_step = 0.1; */
    /*plannerConfig.accuracy = 10.0 / 100.0; */

    /*// Specify affordance screw info */
    /*affordance_util::ScrewInfo aff; */
    /*/1* aff.type = "translation"; *1/ */
    /*/1* aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0); *1/ */
    /*aff.type = "rotation"; */
    /*aff.axis = Eigen::Vector3d(0.0, 0.0, 1.0); */
    /*aff.location = Eigen::Vector3d(0.0, 0.0, 0.0); */
    /*/1* aff.type = "screw"; *1/ */
    /*/1* aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0); *1/ */
    /*/1* aff.location = Eigen::Vector3d(1.0, 2.0, 3.0); *1/ */
    /*/1* aff.pitch = 0.2; *1/ */

    /*// Specify frame name if using apriltag to get affordance locaiton */
    /*/1* aff.location_frame = "affordance_frame"; *1/ */

    /*// Specify EE and gripper orientation goals */
    /*const size_t gripper_control_par = 1; // 1 for affordance control only, 2 for affordance plus gripper x
       orientation, */
    /*                                      // 3 for affordance and gripper xy and so on. Set goals accordingly. */
    /*Eigen::VectorXd goal(gripper_control_par); */
    /*const double aff_goal = 1.5 * M_PI; */
    /*goal.tail(1)(0) = aff_goal; // End element */
    /*/1* goal[0] = 0.1; // EE orientation goal *1/ */
    /*/1*********************************************************/

    /*// Run the planner */
    /*bool success = node->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par); */

    /*if (success) // If trajectory was successfully executed wait until, goal response callback is invoked and ros
       is */
    /*             // shutdown from there */
    /*{ */
    /*    spinner_thread.join(); // join the spinning thread and exit */
    /*} */
    /*else */
    /*{ */
    /*    rclcpp::shutdown();    // shutdown ROS on failure */
    /*    spinner_thread.join(); // join the spinner thread */
    /*} */

    return 0;
}
