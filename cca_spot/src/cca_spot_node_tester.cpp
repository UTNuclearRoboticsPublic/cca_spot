#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>
#include <spot_msgs/action/walk_to.hpp>
#include <tf2_ros/buffer.h>

class WalkToAndMoveChair : public cc_affordance_planner_ros::CcAffordancePlannerRos
{
  public:
    using WalkTo = spot_msgs::action::WalkTo;
    using GoalHandleWalkTo = rclcpp_action::ClientGoalHandle<WalkTo>;

    explicit WalkToAndMoveChair(const std::string &node_name, const rclcpp::NodeOptions &node_options)
        : cc_affordance_planner_ros::CcAffordancePlannerRos(node_name, node_options),
          walk_action_server_name_("/spot_driver/walk_to"),
          gripper_open_server_name_("/spot_manipulation_driver/open_gripper"),
          gripper_close_server_name_("/spot_manipulation_driver/close_gripper"),
          mini_unstow_server_name_("/spot_manipulation_driver/mini_unstow_arm"),
          stow_server_name_("/spot_manipulation_driver/stow_arm")
    {
        // Initialize clients
        gripper_open_client_ = this->create_client<std_srvs::srv::Trigger>(gripper_open_server_name_);
        gripper_close_client_ = this->create_client<std_srvs::srv::Trigger>(gripper_close_server_name_);
        mini_unstow_client_ = this->create_client<std_srvs::srv::Trigger>(mini_unstow_server_name_);
        stow_client_ = this->create_client<std_srvs::srv::Trigger>(stow_server_name_);
        walk_action_client_ = rclcpp_action::create_client<spot_msgs::action::WalkTo>(this, walk_action_server_name_);

        // Construct buffer to lookup chair location from apriltag using tf data
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void run_demo()
    {

        if (!walk_to_chair_())
        {
            RCLCPP_ERROR(this->get_logger(), "Walking to chair failed");
            return;
        }

        if (!mini_unstow_arm())
        {

            RCLCPP_ERROR(this->get_logger(), "Mini unstow failed");
            return;
        }

        if (!execute_approach_motion())
        {

            RCLCPP_ERROR(this->get_logger(), "Approach motion failed");
            return;
        }

        rclcpp::Rate loop_rate(4);
        while (*approach_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED)
        {
            if (*approach_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "Approach motion was interrupted mid-execution.");
                return;
            }

            loop_rate.sleep();
        }

        if (!open_gripper())
        {

            RCLCPP_ERROR(this->get_logger(), "Opening gripper failed");
            return;
        }

        if (!execute_grasp_tune_motion())
        {

            RCLCPP_ERROR(this->get_logger(), "Grasp-tune motion failed");
            return;
        }
        while (*grasp_tune_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED)
        {
            if (*grasp_tune_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "Grasp-tune motion was interrupted mid-execution.");
                return;
            }

            loop_rate.sleep();
        }

        if (!close_gripper())
        {

            RCLCPP_ERROR(this->get_logger(), "Closing gripper failed");
            return;
        }

        if (!execute_affordance_motion())
        {

            RCLCPP_ERROR(this->get_logger(), "Affordance motion failed");
            return;
        }
        while (*affordance_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED)
        {
            if (*affordance_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "Affordance motion was interrupted mid-execution.");
                return;
            }

            loop_rate.sleep();
        }
        if (!execute_push_motion())
        {

            RCLCPP_ERROR(this->get_logger(), "Push motion failed");
            return;
        }
        while (*push_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED)
        {
            if (*push_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "Push motion was interrupted mid-execution.");
                return;
            }

            loop_rate.sleep();
        }
        if (!open_gripper())
        {

            RCLCPP_ERROR(this->get_logger(), "Opening gripper failed");
            return;
        }
        if (!execute_retract_motion())
        {

            RCLCPP_ERROR(this->get_logger(), "Retract motion failed");
            return;
        }
        while (*retract_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED)
        {
            if (*retract_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "Retract motion was interrupted mid-execution.");
                return;
            }

            loop_rate.sleep();
        }
        if (!stow_arm())
        {

            RCLCPP_ERROR(this->get_logger(), "Stow failed");
            return;
        }
        if (!close_gripper())
        {

            RCLCPP_ERROR(this->get_logger(), "Closing gripper failed");
            return;
        }
    }

  private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // buffer to lookup tf data
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    // Clients
    rclcpp_action::Client<spot_msgs::action::WalkTo>::SharedPtr walk_action_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_open_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_close_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mini_unstow_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stow_client_;
    // Client names
    std::string walk_action_server_name_;
    std::string gripper_open_server_name_;
    std::string gripper_close_server_name_;
    std::string mini_unstow_server_name_;
    std::string stow_server_name_;

    const std::string walk_ref_frame_ = "map";
    const std::string cc_ref_frame_ = "arm0_base_link";
    const std::string tool_frame_ = "arm0_tool0";
    const std::string chair_frame_ = "affordance_frame"; // Name of the AprilTag frame to locate the chair

    const Eigen::Matrix4d htm_c2wg_ =
        (Eigen::Matrix4d() << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 1.0)
            .finished(); // chair to walk goal

    const Eigen::Matrix4d htm_c2a_ =
        (Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.36, 0.0, 0.0, 1.0, 0.40, 0.0, 0.0, 0.0, 1.0)
            .finished(); // chair to approach

    bool walk_result_available_ = false;
    bool mini_unstow_result_available_ = false;
    bool walk_success_ = false;
    bool mini_unstow_success_ = false;
    std_srvs::srv::Trigger::Request::SharedPtr trigger_req_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::shared_ptr<cc_affordance_planner_ros::Status> approach_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> grasp_tune_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> affordance_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> retract_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> push_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    // Methods
    bool walk_to_chair_()
    {

        //  Lookup and compute walk goal
        const Eigen::Isometry3d htm_wr2c =
            affordance_util_ros::get_htm(walk_ref_frame_, chair_frame_, *tf_buffer_); // walk ref frame to chair
        if (htm_wr2c.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
            return false;
        }

        const Eigen::Matrix4d htm_wr2wg = htm_wr2c.matrix() * htm_c2wg_;

        const Eigen::Quaterniond quat_wr2wg(htm_wr2wg.block<3, 3>(0, 0)); // quaternion representation

        // Fill out walk goal message
        geometry_msgs::msg::PoseStamped walk_goal_pose;
        walk_goal_pose.header.frame_id = walk_ref_frame_;
        walk_goal_pose.header.stamp = this->get_clock()->now();

        walk_goal_pose.pose.position.x = htm_wr2wg(0, 3);
        walk_goal_pose.pose.position.y = htm_wr2wg(1, 3);
        walk_goal_pose.pose.orientation.x = quat_wr2wg.x();
        walk_goal_pose.pose.orientation.y = quat_wr2wg.y();
        walk_goal_pose.pose.orientation.z = quat_wr2wg.z();
        walk_goal_pose.pose.orientation.w = quat_wr2wg.w();

        WalkTo::Goal walk_goal;
        walk_goal.target_pose = walk_goal_pose;
        walk_goal.maximum_movement_time = 10.0;

        using namespace std::chrono_literals;
        using namespace std::placeholders;
        auto send_goal_options = rclcpp_action::Client<WalkTo>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WalkToAndMoveChair::walk_goal_response_callback_, this, _1);
        send_goal_options.result_callback = std::bind(&WalkToAndMoveChair::walk_result_callback_, this, _1);
        this->walk_action_client_->async_send_goal(walk_goal, send_goal_options);
        rclcpp::Rate loop_rate(4);
        while (!walk_result_available_)
        {
            loop_rate.sleep();
        }
        return walk_success_;
    }
    void walk_goal_response_callback_(const GoalHandleWalkTo::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by %s action server", walk_action_server_name_.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by %s action server, waiting for result",
                        walk_action_server_name_.c_str());
        }
    }

    void walk_result_callback_(const GoalHandleWalkTo::WrappedResult &result)
    {
        walk_result_available_ = true;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "%s action server goal was aborted", walk_action_server_name_.c_str());
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "%s action server goal was canceled", walk_action_server_name_.c_str());
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "%s action server unknown result code", walk_action_server_name_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "%s action server call concluded", walk_action_server_name_.c_str());
        walk_success_ = true;
    }

    bool mini_unstow_arm()
    {

        auto result = mini_unstow_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", mini_unstow_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "%s failed to call mini unstow service", mini_unstow_server_name_.c_str());
            return false;
        }
    }

    bool stow_arm()
    {

        auto result = stow_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", stow_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "%s failed to call mini unstow service", stow_server_name_.c_str());
            return false;
        }
    }
    bool open_gripper()
    {

        auto result = gripper_open_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", gripper_open_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "%s failed to call mini unstow service",
                         gripper_open_server_name_.c_str());
            return false;
        }
    }
    bool close_gripper()
    {

        auto result = gripper_close_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", gripper_close_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "%s failed to call mini unstow service",
                         gripper_close_server_name_.c_str());
            return false;
        }
    }

    bool execute_approach_motion()
    {

        // Compute approach screw
        const Eigen::Isometry3d htm_ccr2c = affordance_util_ros::get_htm(
            cc_ref_frame_, chair_frame_, *tf_buffer_); // closed-chain reference frame to chair

        if (htm_ccr2c.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
            return false;
        }

        Eigen::Matrix4d approach_pose = htm_ccr2c.matrix() * htm_c2a_;
        approach_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        const Eigen::Isometry3d start_pose = affordance_util_ros::get_htm(
            cc_ref_frame_, tool_frame_, *tf_buffer_); // closed-chain reference frame to tool frame

        if (start_pose.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", tool_frame_.c_str());
            return false;
        }
        const Eigen::Matrix<double, 6, 1> approach_twist =
            affordance_util::Adjoint(start_pose.matrix()) *
            affordance_util::se3ToVec(
                affordance_util::MatrixLog6(affordance_util::TransInv(start_pose.matrix()) * approach_pose));

        const Eigen::Matrix<double, 6, 1> approach_screw = approach_twist / approach_twist.norm();

        // Configure the planner
        cc_affordance_planner::PlannerConfig plannerConfig;
        plannerConfig.accuracy = 10.0 / 100.0;
        plannerConfig.aff_step = 0.05;

        // Specify EE and gripper orientation goals
        /* const size_t gripper_control_par = 4; */
        const size_t gripper_control_par = 1;
        Eigen::VectorXd goal = Eigen::VectorXd::Zero(gripper_control_par);
        const double aff_goal = approach_twist.norm();
        goal.tail(1)(0) = aff_goal; // End element

        return this->run_cc_affordance_planner(plannerConfig, approach_screw, goal, gripper_control_par,
                                               approach_motion_status_);
    }

    bool execute_grasp_tune_motion()
    {

        const Eigen::VectorXd aff_screw = (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();

        // Configure the planner
        cc_affordance_planner::PlannerConfig plannerConfig;
        plannerConfig.accuracy = 10.0 / 100.0;
        plannerConfig.aff_step = 0.03;

        // Specify EE and gripper orientation goals
        /* const size_t gripper_control_par = 4; */
        const size_t gripper_control_par = 1;
        Eigen::VectorXd goal = Eigen::VectorXd::Zero(gripper_control_par);
        const double aff_goal = 0.14;
        goal.tail(1)(0) = aff_goal; // End element

        return this->run_cc_affordance_planner(plannerConfig, aff_screw, goal, gripper_control_par,
                                               grasp_tune_motion_status_);
    }
    bool execute_affordance_motion()
    {

        affordance_util::ScrewInfo aff;

        aff.type = "rotation";
        aff.axis = Eigen::Vector3d(0, 0, 1);
        aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Configure the planner
        cc_affordance_planner::PlannerConfig plannerConfig;
        plannerConfig.accuracy = 10.0 / 100.0;
        plannerConfig.aff_step = 0.15;

        // Specify EE and gripper orientation goals
        const size_t gripper_control_par = 1;
        Eigen::VectorXd goal = Eigen::VectorXd::Zero(gripper_control_par);
        const double aff_goal = (1.0 / 2.0) * M_PI;
        goal.tail(1)(0) = aff_goal; // End element

        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par,
                                               affordance_motion_status_);
    }

    bool execute_retract_motion()
    {

        const Eigen::VectorXd aff_screw = (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();

        // Configure the planner
        cc_affordance_planner::PlannerConfig plannerConfig;
        plannerConfig.accuracy = 10.0 / 100.0;
        plannerConfig.aff_step = 0.05;

        // Specify EE and gripper orientation goals
        /* const size_t gripper_control_par = 4; */
        const size_t gripper_control_par = 1;
        Eigen::VectorXd goal = Eigen::VectorXd::Zero(gripper_control_par);
        const double aff_goal = 0.16;
        goal.tail(1)(0) = aff_goal; // End element

        return this->run_cc_affordance_planner(plannerConfig, aff_screw, goal, gripper_control_par,
                                               retract_motion_status_);
    }

    bool execute_push_motion()
    {

        const Eigen::VectorXd aff_screw = (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 0.0, -1.0, 0.0).finished();

        // Configure the planner
        cc_affordance_planner::PlannerConfig plannerConfig;
        plannerConfig.accuracy = 10.0 / 100.0;
        plannerConfig.aff_step = 0.02;

        // Specify EE and gripper orientation goals
        /* const size_t gripper_control_par = 4; */
        const size_t gripper_control_par = 1;
        Eigen::VectorXd goal = Eigen::VectorXd::Zero(gripper_control_par);
        const double aff_goal = 0.08;
        goal.tail(1)(0) = aff_goal; // End element

        return this->run_cc_affordance_planner(plannerConfig, aff_screw, goal, gripper_control_par,
                                               push_motion_status_);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    /* auto node = std::make_shared<CcAffordancePlannerRos>("cc_affordance_planner_ros", node_options); */
    auto node = std::make_shared<WalkToAndMoveChair>("cc_affordance_planner_ros", node_options);

    // Start spinning the node in a separate thread so we could do things like reading parameters and joint states
    // inside the node
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });

    /*// ROS client declaration and initializations */

    /*// Configure the planner */
    /*cc_affordance_planner::PlannerConfig plannerConfig; */
    /*plannerConfig.accuracy = 10.0 / 100.0; */
    /*/1* plannerConfig.aff_step = 0.05; // drawer_tasks *1/ */
    /*/1* plannerConfig.aff_step = 0.15; // moving_a_stool *1/ */
    /*plannerConfig.aff_step = 0.2; // valve_turn experiments */

    /*// Specify affordance screw info */
    /*affordance_util::ScrewInfo aff; */

    /*// Drawer experiments */
    /*/1* aff.type = "translation"; *1/ */
    /*/1* aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0); // pulling_a_drawer *1/ */
    /*/1* aff.axis = Eigen::Vector3d(-1.0, 0.0, 0.0); // pushing_a_drawer *1/ */
    /*// Valve and stool experiments */
    /*aff.type = "rotation"; */
    /*/1* aff.axis = Eigen::Vector3d(0, 1, 0);                            // valve_turn_case_1_and_2 *1/ */
    /*aff.axis = Eigen::Vector3d(1, 0, 0); // valve_turn_case_3_and_4 */
    /*/1* aff.axis = Eigen::Vector3d(0, 0, 1); // moving_a_stool *1/ */
    /*const std::string vir_screw_order = "yzx"; // valve_turn_case_3_and_4 */
    /*/1* const std::string vir_screw_order = "zxy";                      // valve_turn_case_1_and_2 *1/ */
    /*/1* const std::string vir_screw_order = "xyz"; // default *1/ */
    /*aff.location = Eigen::Vector3d(0.616291, 0.0574117, 0.225832); // valve_turn_case_4 */
    /*/1* aff.location = Eigen::Vector3d(0.617247, 0.0635829, 0.224735);  // valve_turn_case_3 *1/ */
    /*/1* aff.location = Eigen::Vector3d(0.626239, -0.0652102, 0.222272); // valve_turn_case_2 *1/ */
    /*/1* aff.location = Eigen::Vector3d(0.693979, -0.0252093, 0.219907); // valve_turn_case_1 *1/ */
    /*/1* aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);                  // moving_a_stool *1/ */

    /*// Specify EE and gripper orientation goals */
    /*/1* const size_t gripper_control_par = 1; // pulling_a_drawer *1/ */
    /*/1* const size_t gripper_control_par = 1; // pushing_a_drawer *1/ */
    /*/1* const size_t gripper_control_par = 1; // moving_a_stool *1/ */
    /*const size_t gripper_control_par = 1; // valve_turn_case_4 */
    /*/1* const size_t gripper_control_par = 2; // valve_turn_case_3 *1/ */
    /*/1* const size_t gripper_control_par = 1; // valve_turn_case_2 *1/ */
    /*/1* const size_t gripper_control_par = 2; // valve_turn_case_1 *1/ */
    /*Eigen::VectorXd goal(gripper_control_par); */
    /*/1* const double aff_goal = 0.29; // pulling_a_drawer *1/ */
    /*/1* const double aff_goal = 0.1999; // pushing_a_drawer *1/ */
    /*/1* const double aff_goal = (1.0 / 2.0) * M_PI; // moving_a_stool *1/ */
    /*const double aff_goal = (7.0 / 3.0) * M_PI; // valve_turn_case_4 */
    /*/1* const double aff_goal = (1.75) * M_PI;      // valve_turn_case_3 *1/ */
    /*/1* const double aff_goal = (1.0 / 4.0) * M_PI; // valve_turn_case_2 *1/ */
    /*/1* const double aff_goal = (1.0 / 2.0) * M_PI; // valve_turn_case_1 *1/ */
    /*goal.tail(1)(0) = aff_goal; // End element */
    /*/1* goal[0] = 0.1; // EE orientation goal *1/ */

    /*// Start states for offline-planning purposes */
    /*Eigen::VectorXd robot_start_config(6); */
    /*/1* robot_start_config << -0.00076, -0.87982, 1.73271, 0.01271, -1.13217, -0.00273; // pulling_a_drawer *1/ */
    /*/1* robot_start_config << 0.00795, -1.18220, 2.46393, 0.02025, -1.32321, -0.00053; // pushing a drawer *1/ */
    /*/1* robot_start_config << 0.20841, -0.52536, 1.85988, 0.18575, -1.37188, -0.07426; // moving a stool *1/ */
    /*robot_start_config << -0.1408, -1.5098, 1.8449, -0.2122, -0.2909, -0.1452; // valve_turn_case_4 */
    /*/1* robot_start_config << 0.0177, -1.2799, 2.1361, 0.0438, -0.8449, -0.0771; // valve_turn_case_3 *1/ */
    /*/1* robot_start_config << 0.0140, -2.0268, 2.1910, 0.8096, -0.1487, 0.8129; // valve_turn_case_2 *1/ */
    /*/1* robot_start_config << 0.1513, -1.8212, 2.0578, 0.8435, -0.3443, 0.7953; // valve_turn_case_1 *1/ */

    /**********************************************************/

    /*// Run the planner */
    /*bool success = node->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
     */
    /*                                               robot_start_config); */

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

    node->run_demo();
    /* std::string conf; */
    /* std::cin >> conf; */

    /* if (success) // If trajectory was successfully executed wait until, goal response callback is invoked and ros
     * is
     */
    // shutdown from there
    /* { */
    /* std::cout << "Walking was a success" << std::endl; */
    spinner_thread.join(); // join the spinning thread and exit
                           /* } */
                           /* else */
                           /* { */
    /* std::cout << "Walking failed" << std::endl; */
    rclcpp::shutdown(); // shutdown ROS on failure
    /* spinner_thread.join(); // join the spinner thread */
    /* } */
    return 0;
}
