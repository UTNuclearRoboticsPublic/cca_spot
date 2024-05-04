#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
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
    /*------------------------------------------------------------*/
    // Specify affordance screw
    /* const Eigen::Vector3d aff_screw_axis(-1, 0, 0);         // screw axis */
    const Eigen::Vector3d aff_screw_axis(0, 1, 0);          // screw axis
    const Eigen::Vector3d aff_screw_axis_location(0, 0, 0); // location vector
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Set affordance goal
    const double aff_goal = (1.0 / 2.0) * M_PI; // Code
    /* Eigen::VectorXd sec_goal(1); */
    /* Eigen::VectorXd sec_goal(2); */
    /* Eigen::VectorXd sec_goal(3); */
    Eigen::VectorXd sec_goal(4);
    sec_goal[0] = 0.0;
    sec_goal[1] = 0.0;
    sec_goal[2] = 0.0;
    sec_goal[3] = aff_goal;
    /* sec_goal[1] = aff_goal; */
    std::cout << "Here is the secondary goal: \n" << sec_goal << std::endl;
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Optionally set planner parameters
    const double aff_step = 0.1;
    /* const int gripper_control_par_tau = 1; */
    /* const int gripper_control_par_tau = 2; */
    /* const int gripper_control_par_tau = 3; */
    const int gripper_control_par_tau = 4;
    const double accuracy = 10.0 / 100.0;
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Run the planner
    /* bool success = node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal); */
    // Or if getting affordance screw location from apriltag
    const std::string apriltag_frame_name = "affordance_frame";
    bool success = node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, sec_goal, aff_step,
                                                   gripper_control_par_tau, accuracy);
    /* Note screw axis is manually set in this case as aff_screw_axis above.Just the location is gotten from Apriltag */
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Optionally, with planner parameters call:
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal, aff_step,
     * gripper_control_par_tau, accuracy); */
    // or
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal, aff_step,
     * gripper_control_par_tau, accuracy); */
    /*------------------------------------------------------------*/

    // Call the function again with new (or old) aff_screw_axis, aff_screw_axis_location and aff_goal to execute
    // another affordance in series
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal); */
    // or
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal); */
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
