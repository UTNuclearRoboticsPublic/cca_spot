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
    plannerConfig.accuracy = 10.0 / 100.0;
    /* plannerConfig.aff_step = 0.05; // drawer_tasks */
    /* plannerConfig.aff_step = 0.15; // moving_a_stool */
    plannerConfig.aff_step = 0.2; // valve_turn experiments

    // Specify affordance screw info
    affordance_util::ScrewInfo aff;

    // Drawer experiments
    /* aff.type = "translation"; */
    /* aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0); // pulling_a_drawer */
    /* aff.axis = Eigen::Vector3d(-1.0, 0.0, 0.0); // pushing_a_drawer */
    // Valve and stool experiments
    aff.type = "rotation";
    /* aff.axis = Eigen::Vector3d(0, 1, 0);                            // valve_turn_case_1_and_2 */
    aff.axis = Eigen::Vector3d(1, 0, 0); // valve_turn_case_3_and_4
    /* aff.axis = Eigen::Vector3d(0, 0, 1); // moving_a_stool */
    const std::string vir_screw_order = "yzx"; // valve_turn_case_3_and_4
    /* const std::string vir_screw_order = "zxy";                      // valve_turn_case_1_and_2 */
    /* const std::string vir_screw_order = "xyz"; // default */
    aff.location = Eigen::Vector3d(0.616291, 0.0574117, 0.225832); // valve_turn_case_4
    /* aff.location = Eigen::Vector3d(0.617247, 0.0635829, 0.224735);  // valve_turn_case_3 */
    /* aff.location = Eigen::Vector3d(0.626239, -0.0652102, 0.222272); // valve_turn_case_2 */
    /* aff.location = Eigen::Vector3d(0.693979, -0.0252093, 0.219907); // valve_turn_case_1 */
    /* aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);                  // moving_a_stool */

    // Specify EE and gripper orientation goals
    /* const size_t gripper_control_par = 1; // pulling_a_drawer */
    /* const size_t gripper_control_par = 1; // pushing_a_drawer */
    /* const size_t gripper_control_par = 1; // moving_a_stool */
    const size_t gripper_control_par = 1; // valve_turn_case_4
    /* const size_t gripper_control_par = 2; // valve_turn_case_3 */
    /* const size_t gripper_control_par = 1; // valve_turn_case_2 */
    /* const size_t gripper_control_par = 2; // valve_turn_case_1 */
    Eigen::VectorXd goal(gripper_control_par);
    /* const double aff_goal = 0.29; // pulling_a_drawer */
    /* const double aff_goal = 0.1999; // pushing_a_drawer */
    /* const double aff_goal = (1.0 / 2.0) * M_PI; // moving_a_stool */
    const double aff_goal = (7.0 / 3.0) * M_PI; // valve_turn_case_4
    /* const double aff_goal = (1.75) * M_PI;      // valve_turn_case_3 */
    /* const double aff_goal = (1.0 / 4.0) * M_PI; // valve_turn_case_2 */
    /* const double aff_goal = (1.0 / 2.0) * M_PI; // valve_turn_case_1 */
    goal.tail(1)(0) = aff_goal; // End element
    /* goal[0] = 0.1; // EE orientation goal */

    // Start states for offline-planning purposes
    Eigen::VectorXd robot_start_config(6);
    /* robot_start_config << -0.00076, -0.87982, 1.73271, 0.01271, -1.13217, -0.00273; // pulling_a_drawer */
    /* robot_start_config << 0.00795, -1.18220, 2.46393, 0.02025, -1.32321, -0.00053; // pushing a drawer */
    /* robot_start_config << 0.20841, -0.52536, 1.85988, 0.18575, -1.37188, -0.07426; // moving a stool */
    robot_start_config << -0.1408, -1.5098, 1.8449, -0.2122, -0.2909, -0.1452; // valve_turn_case_4
    /* robot_start_config << 0.0177, -1.2799, 2.1361, 0.0438, -0.8449, -0.0771; // valve_turn_case_3 */
    /* robot_start_config << 0.0140, -2.0268, 2.1910, 0.8096, -0.1487, 0.8129; // valve_turn_case_2 */
    /* robot_start_config << 0.1513, -1.8212, 2.0578, 0.8435, -0.3443, 0.7953; // valve_turn_case_1 */

    /*********************************************************/

    // Run the planner
    bool success = node->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
                                                   robot_start_config);

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
