#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <algorithm>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>
#include <spot_msgs/action/walk_to.hpp>
#include <spot_msgs/srv/dock.hpp>
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
          stow_server_name_("/spot_manipulation_driver/stow_arm"),
          undock_server_name_("/spot_driver/undock"),
          dock_server_name_("/spot_driver/dock")
    {
        // Initialize clients
        gripper_open_client_ = this->create_client<std_srvs::srv::Trigger>(gripper_open_server_name_);
        gripper_close_client_ = this->create_client<std_srvs::srv::Trigger>(gripper_close_server_name_);
        mini_unstow_client_ = this->create_client<std_srvs::srv::Trigger>(mini_unstow_server_name_);
        stow_client_ = this->create_client<std_srvs::srv::Trigger>(stow_server_name_);
        undock_client_ = this->create_client<std_srvs::srv::Trigger>(undock_server_name_);
        dock_client_ = this->create_client<spot_msgs::srv::Dock>(dock_server_name_);
        walk_action_client_ = rclcpp_action::create_client<spot_msgs::action::WalkTo>(this, walk_action_server_name_);

        // Construct buffer to lookup chair location from apriltag using tf data
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    ~WalkToAndMoveChair() { rclcpp::shutdown(); }
    void run_demo()
    {
        /********************************************************/
        /* RCLCPP_INFO(this->get_logger(), "Undocking robot"); */
        /* if (!undock_robot()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Undock failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        // Capture robot's current pose to walk back to it later
        /* const Eigen::Isometry3d htm_start_pose = */
            /* affordance_util_ros::get_htm(fixed_frame_, robot_navigation_frame_, *tf_buffer_); */

        /* RCLCPP_INFO(this->get_logger(), "Walking to chair"); */
        /* if (!walk_to_chair_()) */
        /* { */
        /*     RCLCPP_ERROR(this->get_logger(), "Walking to chair failed"); */
        /*     /1* return; *1/ */
        /* } */

        /* walk_result_available_ = false; */
        /* walk_success_ = false; */
        /* RCLCPP_INFO(this->get_logger(), "Walking to chair"); */
        /* if (!walk_to_chair_()) */
        /* { */
        /*     RCLCPP_ERROR(this->get_logger(), "Walking to chair failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Mini-unstowing arm"); */
        /* if (!mini_unstow_arm()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Mini unstow failed"); */
        /*     return; */
        /* } */

        /********************************************************/
        rclcpp::Rate loop_rate(4);
        /* RCLCPP_INFO(this->get_logger(), "Executing pre-approach forward motion"); */
        /* if (!execute_preapproach_forward_motion()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Pre-approach forward motion failed"); */
        /*     return; */
        /* } */
        /* while (*preapproach_forward_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED) */
        /* { */
        /*     if (*preapproach_forward_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN) */
        /*     { */

        /*         RCLCPP_ERROR(this->get_logger(), "Preapproach forward motion was interrupted mid-execution."); */
        /*         return; */
        /*     } */

        /*     loop_rate.sleep(); */
        /* } */

        /********************************************************/

        RCLCPP_INFO(this->get_logger(), "Executing approach motion");
        if (!execute_approach_motion())
        {

            RCLCPP_ERROR(this->get_logger(), "Approach motion failed");
            return;
        }

	/* Eigen::Matrix4d approach_pose; */
	/* approach_pose << 0.998453,  0.0378028,  0.0407843,  0.529228, */
          /* -0.0380367, 0.999264,   0.00497515, -0.16148, */
          /* -0.0405662, -0.00651876,0.999156,   0.100135, */
          /* 0,          0,          0,          1; */
	/* geometry_msgs::msg::TransformStamped t; */
	/* Eigen::Quaterniond approach_pose_quat(approach_pose.block<3,3>(0,0)); */

        /* t.header.stamp = this->get_clock()->now(); */
        /* t.header.frame_id = "base_link"; */
        /* t.child_frame_id = "approach_frame"; */
        /* t.transform.translation.x = approach_pose(0,3); */
        /* t.transform.translation.y = approach_pose(1,3); */
        /* t.transform.translation.z = approach_pose(2,3); */
        /* t.transform.rotation.x = approach_pose_quat.x(); */
        /* t.transform.rotation.y = approach_pose_quat.y(); */
        /* t.transform.rotation.z = approach_pose_quat.z(); */
        /* t.transform.rotation.w = approach_pose_quat.w(); */

        while (*approach_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED)
        {
            if (*approach_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "Approach motion was interrupted mid-execution.");
                return;
            }
	    /* tf_broadcaster_->sendTransform(t); */

            loop_rate.sleep();
        }
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Opening gripper"); */
        /* if (!open_gripper()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Opening gripper failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Executing grasp-tune forward motion"); */
        /* if (!execute_grasp_tune_forward_motion()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Grasp-tune forward motion failed"); */
        /*     return; */
        /* } */
        /* while (*grasp_tune_forward_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED) */
        /* { */
        /*     if (*grasp_tune_forward_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN) */
        /*     { */

        /*         RCLCPP_ERROR(this->get_logger(), "Grasp-tune forward motion was interrupted mid-execution."); */
        /*         return; */
        /*     } */

        /*     loop_rate.sleep(); */
        /* } */

        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Executing grasp-tune upward motion"); */
        /* if (!execute_grasp_tune_upward_motion()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Grasp-tune upward motion failed"); */
        /*     return; */
        /* } */
        /* while (*grasp_tune_upward_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED) */
        /* { */
        /*     if (*grasp_tune_upward_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN) */
        /*     { */

        /*         RCLCPP_ERROR(this->get_logger(), "Grasp-tune upward motion was interrupted mid-execution."); */
        /*         return; */
        /*     } */

        /*     loop_rate.sleep(); */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Closing gripper"); */
        /* if (!close_gripper()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Closing gripper failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Executing affordance motion"); */
        /* if (!execute_affordance_motion()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Affordance motion failed"); */
        /*     return; */
        /* } */
        /* while (*affordance_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED) */
        /* { */
        /*     if (*affordance_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN) */
        /*     { */

        /*         RCLCPP_ERROR(this->get_logger(), "Affordance motion was interrupted mid-execution."); */
        /*         return; */
        /*     } */

        /*     loop_rate.sleep(); */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Executing push motion"); */
        /* if (!execute_push_motion()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Push motion failed"); */
        /*     return; */
        /* } */
        /* while (*push_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED) */
        /* { */
        /*     if (*push_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN) */
        /*     { */

        /*         RCLCPP_ERROR(this->get_logger(), "Push motion was interrupted mid-execution."); */
        /*         return; */
        /*     } */

        /*     loop_rate.sleep(); */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Opening gripper"); */
        /* if (!open_gripper()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Opening gripper failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Executing retract motion"); */
        /* if (!execute_retract_motion()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Retract motion failed"); */
        /*     return; */
        /* } */
        /* while (*retract_motion_status_ != cc_affordance_planner_ros::Status::SUCCEEDED) */
        /* { */
        /*     if (*retract_motion_status_ == cc_affordance_planner_ros::Status::UNKNOWN) */
        /*     { */

        /*         RCLCPP_ERROR(this->get_logger(), "Retract motion was interrupted mid-execution."); */
        /*         return; */
        /*     } */

        /*     loop_rate.sleep(); */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Stowing arm"); */
        /* if (!stow_arm()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Stow failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        /* if (!close_gripper()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Closing gripper failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        /* walk_result_available_ = false; */
        /* walk_success_ = false; */
        /* RCLCPP_INFO(this->get_logger(), "Walking back to start pose"); */
        /* if (!walk_back_to_start_pose_(htm_start_pose.matrix())) */
        /* { */
        /*     RCLCPP_ERROR(this->get_logger(), "Walking back to start pose failed"); */
        /*     return; */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Docking robot"); */
        /* if (!dock_robot()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Dock failed"); */
        /*     return; */
        /* } */

        /********************************************************/
        rclcpp::shutdown();
    }

  private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // buffer to lookup tf data
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Clients
    rclcpp_action::Client<spot_msgs::action::WalkTo>::SharedPtr walk_action_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_open_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_close_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mini_unstow_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stow_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undock_client_;
    rclcpp::Client<spot_msgs::srv::Dock>::SharedPtr dock_client_;
    // Client names
    std::string walk_action_server_name_;
    std::string gripper_open_server_name_;
    std::string gripper_close_server_name_;
    std::string mini_unstow_server_name_;
    std::string stow_server_name_;
    std::string undock_server_name_;
    std::string dock_server_name_;

    const std::string ref_frame_ = "arm0_base_link";
    const std::string tool_frame_ = "arm0_tool0";
    const std::string chair_frame_ = "affordance_frame"; // Name of the AprilTag frame to locate the chair
    const std::string fixed_frame_ = "odom";
    const std::string robot_navigation_frame_ = "base_footprint";

    const Eigen::Matrix4d htm_c2wg_ =
        (Eigen::Matrix4d() << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 1.4, 0.0, 0.0, 0.0, 1.0)
            .finished(); // chair to walk goal

    const Eigen::Matrix4d htm_c2a_ =
        (Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.36, 0.0, 0.0, 1.0, 0.50, 0.0, 0.0, 0.0, 1.0)
            .finished(); // chair to approach
    const double approach_pose_z_offset_ = -0.3;

    bool walk_result_available_ = false;
    bool walk_success_ = false;
    std_srvs::srv::Trigger::Request::SharedPtr trigger_req_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::shared_ptr<cc_affordance_planner_ros::Status> preapproach_forward_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> approach_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> grasp_tune_forward_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> grasp_tune_upward_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> affordance_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> push_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    std::shared_ptr<cc_affordance_planner_ros::Status> retract_motion_status_ =
        std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN);
    // Methods
    bool undock_robot()
    {

        // Wait for service to be available
        while (!undock_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             undock_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...", undock_server_name_.c_str());
        }

        auto result = undock_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", undock_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", undock_server_name_.c_str());
            return false;
        }
    }

    bool walk_to_chair_()
    {

        //  Lookup and compute walk goal
        rclcpp::Rate tag_read_rate(4);
        std::vector<Eigen::Matrix4d> htm_r2c_vec;
        for (int i = 0; i < 10; i++)
        {
            const Eigen::Isometry3d htm_r2c =
                affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // ref frame to chair
            if (htm_r2c.matrix().isApprox(Eigen::Matrix4d::Identity()))
            {
                RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
                return false;
            }
            affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // ref frame to chair
            htm_r2c_vec.push_back(htm_r2c.matrix());
            tag_read_rate.sleep();
        }
        Eigen::Matrix4d htm_r2c_matrix = findMedianNormMatrix(htm_r2c_vec);
        if (htm_r2c_matrix.isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "The median matrix is close to identity");
            return false;
        }

        const Eigen::Matrix4d htm_wr2wg = htm_r2c_matrix * htm_c2wg_;
        /* const Eigen::Matrix4d htm_wr2wg = htm_r2c.matrix() * htm_c2wg_; */

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

        // Wait for service to be available
        while (!mini_unstow_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             mini_unstow_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...",
                        mini_unstow_server_name_.c_str());
        }

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
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", mini_unstow_server_name_.c_str());
            return false;
        }
    }

    bool stow_arm()
    {

        // Wait for service to be available
        while (!stow_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             stow_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...", stow_server_name_.c_str());
        }

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
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", stow_server_name_.c_str());
            return false;
        }
    }
    bool open_gripper()
    {

        // Wait for service to be available
        while (!gripper_open_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             gripper_open_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...",
                        gripper_open_server_name_.c_str());
        }

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
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", gripper_open_server_name_.c_str());
            return false;
        }
    }
    bool close_gripper()
    {

        // Wait for service to be available
        while (!gripper_close_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             gripper_close_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...",
                        gripper_close_server_name_.c_str());
        }

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
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", gripper_close_server_name_.c_str());
            return false;
        }
    }

    bool execute_approach_motion()
    {

        // Compute approach screw
        /* const Eigen::Isometry3d htm_r2c = */
        /*     affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // reference frame to chair */

        /* if (htm_r2c.matrix().isApprox(Eigen::Matrix4d::Identity())) */
        /* { */
        /*     RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str()); */
        /*     return false; */
        /* } */

        /* Eigen::Matrix4d approach_pose = htm_r2c.matrix() * htm_c2a_;            // adjust y offset in the chair frame */
        /* approach_pose(2, 3) = htm_r2c.matrix()(2, 3) + approach_pose_z_offset_; // adjust z offset in the ref frame */
        /* approach_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); */
	Eigen::Matrix4d approach_pose;
	approach_pose << 0.998453,  0.0378028,  0.0407843,  0.529228,
          -0.0380367, 0.999264,   0.00497515, -0.16148,
          -0.0405662, -0.00651876,0.999156,   0.100135,
          0,          0,          0,          1;
	geometry_msgs::msg::TransformStamped t;
	Eigen::Quaterniond approach_pose_quat(approach_pose.block<3,3>(0,0));

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "arm0_base_link";
        t.child_frame_id = "approach_frame";
        t.transform.translation.x = approach_pose(0,3);
        t.transform.translation.y = approach_pose(1,3);
        t.transform.translation.z = approach_pose(2,3);
        t.transform.rotation.x = approach_pose_quat.x();
        t.transform.rotation.y = approach_pose_quat.y();
        t.transform.rotation.z = approach_pose_quat.z();
        t.transform.rotation.w = approach_pose_quat.w();

	rclcpp::Rate loop_rate(4); 
	for (int i=0;i<=10; i++){
	    tf_broadcaster_->sendTransform(t);
	    loop_rate.sleep();
	
	}
	Eigen::VectorXd robot_start_config(6);
	/* robot_start_config  << -0.433123, */
	robot_start_config  << 0,
           /* -1.49419, */
           -1.09419,
           2.2496,
           -0.567882,
           -0.796551,
           0.396139;


        // Fill out affordance info
        affordance_util::ScrewInfo aff;
        aff.type = "rotation";
        /* aff.axis = Eigen::Vector3d(0, 0, 1); */
        aff.axis = Eigen::Vector3d(1, 0, 0);
        aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);

        // Configure the planner
        cc_affordance_planner::PlannerConfig plannerConfig;
        plannerConfig.accuracy = 10.0 / 100.0;
        plannerConfig.aff_step = 0.05;

        // Specify EE and gripper orientation goals
        const size_t gripper_control_par = 5;
        Eigen::VectorXd goal = Eigen::VectorXd::Zero(gripper_control_par);
        const double aff_goal = (1.0 / 2.0) * M_PI;
	/* goal << 0,0,0,0,aff_goal; */
	/* goal << 0,0,0,1.57,0.0; */
	/* goal << 0,0,-0.57,1.57,0.0; */
	goal << 0,0,0.0,0.0,0.0;
	/* goal << 0,0,0.0,0.0,1.57; */


        const std::string vir_screw_order = "xyz";
        RCLCPP_INFO_STREAM(this->get_logger(), "Here is the goal to the planner ros\n"<<goal);

        /* return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order, */
        /*                                        approach_motion_status_); */
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, approach_pose, gripper_control_par,  vir_screw_order,
                                               approach_motion_status_, robot_start_config);
    }

    bool execute_preapproach_forward_motion()
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
        const double aff_goal = 0.24;
        goal.tail(1)(0) = aff_goal; // End element

        const std::string vir_screw_order = "xyz";
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
                                               preapproach_forward_motion_status_);
    }

    bool execute_grasp_tune_forward_motion()
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
        const double aff_goal = 0.25;
        goal.tail(1)(0) = aff_goal; // End element

        const std::string vir_screw_order = "xyz";
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
                                               grasp_tune_forward_motion_status_);
    }

    bool execute_grasp_tune_upward_motion()
    {

        // Fill out affordance info
        affordance_util::ScrewInfo aff;
        aff.type = "translation";
        aff.axis = Eigen::Vector3d(0.0, 0.0, 1.0);

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

        const std::string vir_screw_order = "none";
        return this->run_cc_affordance_planner(plannerConfig, aff, goal, gripper_control_par, vir_screw_order,
                                               grasp_tune_upward_motion_status_);
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

    bool walk_back_to_start_pose_(const Eigen::Matrix4d &htm_start_pose)
    {

        //  Lookup and compute walk goal
        const Eigen::Quaterniond quat_start_pose(htm_start_pose.block<3, 3>(0, 0)); // quaternion representation

        // Fill out walk goal message
        geometry_msgs::msg::PoseStamped walk_goal_pose;
        walk_goal_pose.header.frame_id = fixed_frame_;
        walk_goal_pose.header.stamp = this->get_clock()->now();

        walk_goal_pose.pose.position.x = htm_start_pose(0, 3);
        walk_goal_pose.pose.position.y = htm_start_pose(1, 3);
        walk_goal_pose.pose.orientation.x = quat_start_pose.x();
        walk_goal_pose.pose.orientation.y = quat_start_pose.y();
        walk_goal_pose.pose.orientation.z = quat_start_pose.z();
        walk_goal_pose.pose.orientation.w = quat_start_pose.w();

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

    bool dock_robot()
    {

        // Wait for service to be available
        while (!dock_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             dock_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...", dock_server_name_.c_str());
        }

        auto dock_req = std::make_shared<spot_msgs::srv::Dock::Request>();
        dock_req->dock_id = 520;
        auto result = dock_client_->async_send_request(dock_req);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", dock_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", dock_server_name_.c_str());
            return false;
        }
    }
    // Function to compute the Frobenius norm of a matrix
    double computeFrobeniusNorm(const Eigen::Matrix4d &matrix) { return matrix.norm(); }

    // Function to find the matrix closest to the median norm
    Eigen::Matrix4d findMedianNormMatrix(const std::vector<Eigen::Matrix4d> &matrices)
    {
        if (matrices.empty())
        {
            throw std::invalid_argument("The matrix list is empty.");
        }

        std::vector<double> norms;
        norms.reserve(matrices.size());

        for (const auto &matrix : matrices)
        {
            norms.push_back(computeFrobeniusNorm(matrix));
        }

        // Sort norms and find the median
        std::sort(norms.begin(), norms.end());
        double medianNorm;
        size_t numMatrices = norms.size();
        if (numMatrices % 2 == 0)
        {
            medianNorm = (norms[numMatrices / 2 - 1] + norms[numMatrices / 2]) / 2.0;
        }
        else
        {
            medianNorm = norms[numMatrices / 2];
        }

        // Find the matrix whose norm is closest to the median norm
        double minDiff = std::numeric_limits<double>::max();
        Eigen::Matrix4d closestMatrix;

        for (const auto &matrix : matrices)
        {
            double norm = computeFrobeniusNorm(matrix);
            double diff = std::abs(norm - medianNorm);

            if (diff < minDiff)
            {
                minDiff = diff;
                closestMatrix = matrix;
            }
        }

        return closestMatrix;
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
