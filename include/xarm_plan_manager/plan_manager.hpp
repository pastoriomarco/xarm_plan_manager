#ifndef PLAN_MANAGER_HPP
#define PLAN_MANAGER_HPP

#include <signal.h>
#include <thread>
#include <algorithm>  // For std::clamp if needed
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_float32_list.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <vector>
#include <string>

// Define a constant for service call failure
#define SERVICE_CALL_FAILED 999

// Forward declaration of the PlanManager class
class PlanManager;

// Global variables
extern std::shared_ptr<rclcpp::Node> node;
extern std::mutex joint_mutex;
extern sensor_msgs::msg::JointState::SharedPtr current_joint_state;

// Type trait to check if a type has a 'success' member
template <typename T, typename = void>
struct has_success : std::false_type {};

template <typename T>
struct has_success<T, std::__void_t<decltype(std::declval<T>().success)>> : std::true_type {};

// Function declarations
template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest req);

int set_scaling_factors(rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr client, const std::vector<float>& datas);

// Callback function for /joint_states subscriber
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

// Helper functions to check targets
bool checkJointTargets(const std::vector<std::string>& joint_names_ordered,
                       const std::vector<double>& target_joint_positions,
                       double tolerance = 0.05);

bool checkPoseTarget(const geometry_msgs::msg::Pose& target_pose,
                    tf2_ros::Buffer& tf_buffer,
                    double position_tolerance = 0.05,
                    double orientation_tolerance = 0.05);

// Signal handler for graceful shutdown
void exit_sig_handler(int signum);

// PlanManager class definition
class PlanManager {
public:
    PlanManager(rclcpp::Node::SharedPtr node, int dof, 
               std::chrono::seconds joint_tout = std::chrono::seconds(30),
               std::chrono::seconds pose_tout = std::chrono::seconds(30),
               int retries = 3);

    bool executeJointMovement(
        const std::vector<double>& target_joint_positions,
        const std::vector<float>& scaling_factors
    );

    bool executePoseMovement(
        const geometry_msgs::msg::Pose& target_pose,
        const std::vector<float>& scaling_factors
    );

    const std::vector<std::string>& getJointNames() const;

private:
    // Fixed parameters
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;
    rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr set_scaling_factors_client_;
    std::chrono::seconds joint_timeout_;
    std::chrono::seconds pose_timeout_;
    int max_retries_;
    
    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Joint names ordered based on DOF
    std::vector<std::string> joint_names_ordered_;

    // Helper method to initialize joint_names_ordered_ based on dof
    void initializeJointNames(int dof);
};

// Template function implementations must be in the header
template<typename ServiceT, typename SharedRequest>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "Service %s not available, waiting...", client->get_service_name());
        }
    }

    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }

    auto res = result_future.get();

    // Use the has_success trait to determine how to handle the response
    if constexpr (has_success<typename ServiceT::Response>::value) {
        RCLCPP_INFO(node->get_logger(), "Called service %s, success=%d", client->get_service_name(), res->success);
        return res->success ? 0 : 1;
    } else {
        RCLCPP_INFO(node->get_logger(), "Called service %s, ret=%d, message=%s",
                    client->get_service_name(), res->ret, res->message.c_str());
        return res->ret;
    }
}

#endif // PLAN_MANAGER_HPP
