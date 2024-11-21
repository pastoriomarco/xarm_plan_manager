#ifndef PLAN_MANAGER_HPP
#define PLAN_MANAGER_HPP

#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_float32_list.hpp>
#include <xarm_msgs/srv/call.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Define a constant for service call failure
#define SERVICE_CALL_FAILED 999

// Type trait to check if a type has a 'success' member
template <typename T, typename = void>
struct has_success : std::false_type {};

template <typename T>
struct has_success<T, std::__void_t<decltype(std::declval<T>().success)>> : std::true_type {};

// Forward declaration of the PlanManager class
class PlanManager;

/**
 * @brief PlanManager class for managing robot arm movements.
 */
class PlanManager {
public:
    /**
     * @brief Constructor for PlanManager.
     * 
     * @param node Shared pointer to the ROS2 node.
     * @param dof Degrees of freedom for the robot arm (supported: 5, 6, 7).
     * @param joint_tout Timeout for joint movements.
     * @param pose_tout Timeout for pose movements.
     * @param retries Number of retry attempts for service calls.
     */
    PlanManager(rclcpp::Node::SharedPtr node, int dof, 
               std::chrono::seconds joint_tout = std::chrono::seconds(30),
               std::chrono::seconds pose_tout = std::chrono::seconds(30),
               int retries = 3);

    /**
     * @brief Destructor for PlanManager.
     */
    ~PlanManager();

    /**
     * @brief Execute joint movement to target positions with scaling factors.
     * 
     * @param target_joint_positions Desired joint positions.
     * @param scaling_factors Scaling factors for the movement.
     * @return true if movement was successful, false otherwise.
     */
    bool executeJointMovement(
        const std::vector<double>& target_joint_positions,
        const std::vector<float>& scaling_factors
    );

    /**
     * @brief Execute pose movement to target pose with scaling factors.
     * 
     * @param target_pose Desired end-effector pose.
     * @param scaling_factors Scaling factors for the movement.
     * @return true if movement was successful, false otherwise.
     */
    bool executePoseMovement(
        const geometry_msgs::msg::Pose& target_pose,
        const std::vector<float>& scaling_factors
    );

    /**
     * @brief Execute linear movement to target pose with scaling factors.
     * 
     * @param target_pose Desired end-effector pose.
     * @param scaling_factors Scaling factors for the movement.
     * @return true if movement was successful, false otherwise.
     */
    bool executeLinearMovement(
        const geometry_msgs::msg::Pose& target_pose,
        const std::vector<float>& scaling_factors
    );

    /**
     * @brief Check if current joint states are within tolerance of target positions.
     * 
     * @param joint_names_ordered Ordered list of joint names.
     * @param target_joint_positions Desired joint positions.
     * @param tolerance Position tolerance.
     * @return true if all joints are within tolerance, false otherwise.
     */
    bool checkJointTargets(const std::vector<std::string>& joint_names_ordered,
                           const std::vector<double>& target_joint_positions,
                           double tolerance = 0.05);

    /**
     * @brief Check if current pose is within tolerance of target pose.
     * 
     * @param target_pose Desired end-effector pose.
     * @param position_tolerance Position tolerance.
     * @param orientation_tolerance Orientation tolerance.
     * @return true if pose is within tolerance, false otherwise.
     */
    bool checkPoseTarget(const geometry_msgs::msg::Pose& target_pose,
                        double position_tolerance = 0.05,
                        double orientation_tolerance = 0.05);

    /**
     * @brief Get the ordered list of joint names.
     * 
     * @return const reference to the joint names vector.
     */
    const std::vector<std::string>& getJointNames() const;

    /**
     * @brief Callback function for /joint_states subscriber.
     * 
     * @param msg Shared pointer to the JointState message.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
    // Member variables
    rclcpp::Node::SharedPtr node_;
    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    rclcpp::Client<xarm_msgs::srv::PlanJoint>::SharedPtr joint_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanPose>::SharedPtr pose_plan_client_;
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_plan_client_;
    rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr set_scaling_factors_client_;
    // Client for linear movements
    rclcpp::Client<xarm_msgs::srv::PlanSingleStraight>::SharedPtr linear_plan_client_;
    // Clients for base and eef links
    rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr get_base_link_client_;
    rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr get_eef_link_client_;
    
    std::chrono::seconds joint_timeout_;
    std::chrono::seconds pose_timeout_;
    int max_retries_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Joint names ordered based on DOF
    std::vector<std::string> joint_names_ordered_;

    // Retrieved base and eef links
    std::string base_link_;
    std::string eef_link_;

    /**
     * @brief Initialize the ordered list of joint names based on DOF.
     * 
     * @param dof Degrees of freedom.
     */
    void initializeJointNames(int dof);

    /**
     * @brief Retrieve base and end-effector links via service calls.
     * 
     * @return true if successful, false otherwise.
     */
    bool retrieveBaseAndEefLinks();

    /**
     * @brief Template function to call a service and handle the response.
     * 
     * @tparam ServiceT Service type.
     * @tparam SharedRequest Shared pointer to the service request type.
     * @param client Shared pointer to the service client.
     * @param req Shared pointer to the service request.
     * @return int Result code based on service response.
     */
    template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
    int call_request(std::shared_ptr<ServiceT> client, SharedRequest req);

    /**
     * @brief Set scaling factors via the corresponding service.
     * 
     * @param client Shared pointer to the SetFloat32List service client.
     * @param datas Scaling factors data.
     * @return int Result code based on service response.
     */
    int set_scaling_factors(rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr client, const std::vector<float>& datas);
};

// Template function implementations must be in the header
template<typename ServiceT, typename SharedRequest>
int PlanManager::call_request(std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node_->get_logger(), "Service %s not available, waiting...", client->get_service_name());
        }
    }

    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }

    auto res = result_future.get();

    // Use the has_success trait to determine how to handle the response
    if constexpr (has_success<typename ServiceT::Response>::value) {
        RCLCPP_INFO(node_->get_logger(), "Called service %s, success=%d", client->get_service_name(), res->success);
        return res->success ? 0 : 1;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Called service %s, ret=%d, message=%s",
                    client->get_service_name(), res->ret, res->message.c_str());
        return res->ret;
    }
}

#endif // PLAN_MANAGER_HPP
