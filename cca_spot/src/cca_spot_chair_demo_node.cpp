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

    ~WalkToAndMoveChair() { rclcpp::shutdown(); }
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

    const std::string ref_frame_ = "arm0_base_link";
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
        const Eigen::Isometry3d htm_r2c =
            affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // ref frame to chair
        if (htm_r2c.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
            return false;
        }

        const Eigen::Matrix4d htm_wr2wg = htm_r2c.matrix() * htm_c2wg_;

        const Eigen::Quaterniond quat_wr2wg(htm_wr2wg.block<3, 3>(0, 0)); // quaternion representation

        // Fill out walk goal message
        geometry_msgs::msg::PoseStamped walk_goal_pose;
        walk_goal_pose.header.frame_id = ref_frame_;
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
        const Eigen::Isometry3d htm_r2c =
            affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // reference frame to chair

        if (htm_r2c.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
            return false;
        }

        Eigen::Matrix4d approach_pose = htm_r2c.matrix() * htm_c2a_;
        approach_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        const Eigen::Isometry3d start_pose =
            affordance_util_ros::get_htm(ref_frame_, tool_frame_, *tf_buffer_); // reference frame to tool frame

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

        // Fill out affordance info
        affordance_util::ScrewInfo aff;
        aff.type = "screw";
        aff.screw = approach_screw;

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

        const std::string vir_screw_order = "none";

        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
                                               approach_motion_status_);
    }

    bool execute_grasp_tune_motion()
    {

        // Fill out affordance info
        affordance_util::ScrewInfo aff;
        aff.type = "translation";
        aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0);

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

        const std::string vir_screw_order = "xyz";
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
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

        const std::string vir_screw_order = "xyz";
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
                                               affordance_motion_status_);
    }

    bool execute_retract_motion()
    {

        // Fill out affordance info
        affordance_util::ScrewInfo aff;
        aff.type = "translation";
        aff.axis = Eigen::Vector3d(0.0, 1.0, 0.0);

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

        const std::string vir_screw_order = "xyz";
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
                                               retract_motion_status_);
    }

    bool execute_push_motion()
    {

        // Fill out affordance info
        affordance_util::ScrewInfo aff;
        aff.type = "translation";
        aff.axis = Eigen::Vector3d(0.0, -1.0, 0.0);

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

        const std::string vir_screw_order = "xyz";
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
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

    // Run the demo
    node->run_demo();
    spinner_thread.join(); // join the spinning thread and exit
    return 0;
}
