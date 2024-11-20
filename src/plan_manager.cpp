// src/xarm_plan_manager/src/plan_manager.cpp

#include "xarm_plan_manager/plan_manager.hpp"

// Initialize the global node pointer
std::shared_ptr<rclcpp::Node> node;

// Implementation of set_scaling_factors
int set_scaling_factors(rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr client, const std::vector<float>& datas)
{
    auto request = std::make_shared<xarm_msgs::srv::SetFloat32List::Request>();
    request->datas = datas;

    // Use the existing call_request function
    int result = call_request(client, request);
    return result;
}

// Implementation of jointStateCallback
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_mutex);
    current_joint_state = msg;
}

// Implementation of checkJointTargets
bool PlanManager::checkJointTargets(const std::vector<std::string>& joint_names_ordered,
                       const std::vector<double>& target_joint_positions,
                       double tolerance)
{
    std::lock_guard<std::mutex> lock(joint_mutex);
    if (!current_joint_state) {
        RCLCPP_WARN(node->get_logger(), "No joint state received yet.");
        return false;
    }

    // Create a map from joint name to position for quick lookup
    std::unordered_map<std::string, double> current_joints_map;
    for (size_t i = 0; i < current_joint_state->name.size(); ++i) {
        current_joints_map[current_joint_state->name[i]] = current_joint_state->position[i];
    }

    // Iterate over the ordered joint names and compare positions
    for (size_t i = 0; i < joint_names_ordered.size(); ++i) {
        const std::string& joint_name = joint_names_ordered[i];
        if (current_joints_map.find(joint_name) == current_joints_map.end()) {
            RCLCPP_WARN(node->get_logger(), "Joint name %s not found in current joint states.", joint_name.c_str());
            return false;
        }
        double current = current_joints_map[joint_name];
        double target = target_joint_positions[i];
        if (std::abs(current - target) > tolerance) {
            // For debugging: Log which joint is not within tolerance
            RCLCPP_DEBUG(node->get_logger(), "Joint %s: current=%.4f, target=%.4f", joint_name.c_str(), current, target);
            return false;
        }
    }
    return true;
}

// Implementation of checkPoseTarget
bool PlanManager::checkPoseTarget(const geometry_msgs::msg::Pose& target_pose,
                    tf2_ros::Buffer& tf_buffer,
                    double position_tolerance,
                    double orientation_tolerance)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        // Use dynamic base and eef links
        transformStamped = tf_buffer.lookupTransform(base_link_, eef_link_, tf2::TimePointZero, std::chrono::seconds(1));
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }

    // Normalize quaternions
    tf2::Quaternion q_current(transformStamped.transform.rotation.x,
                              transformStamped.transform.rotation.y,
                              transformStamped.transform.rotation.z,
                              transformStamped.transform.rotation.w);
    q_current.normalize();

    tf2::Quaternion q_target(target_pose.orientation.x,
                             target_pose.orientation.y,
                             target_pose.orientation.z,
                             target_pose.orientation.w);
    q_target.normalize();

    // Compute the angle difference
    double dot_product = q_current.x() * q_target.x() +
                         q_current.y() * q_target.y() +
                         q_current.z() * q_target.z() +
                         q_current.w() * q_target.w();
    double angle_diff = 2 * acos(std::abs(dot_product));

    // Compare positions
    if (std::abs(transformStamped.transform.translation.x - target_pose.position.x) > position_tolerance ||
        std::abs(transformStamped.transform.translation.y - target_pose.position.y) > position_tolerance ||
        std::abs(transformStamped.transform.translation.z - target_pose.position.z) > position_tolerance) {
        RCLCPP_DEBUG(node->get_logger(), "Pose position difference exceeds tolerance.");
        return false;
    }

    // Compare orientations (quaternions)
    if (angle_diff > orientation_tolerance) {
        RCLCPP_DEBUG(node->get_logger(), "Pose orientation difference exceeds tolerance.");
        return false;
    }

    return true;
}

// Implementation of exit_sig_handler
void exit_sig_handler(int signum)
{
    (void)signum;
    fprintf(stderr, "[plan_manager_node] Ctrl-C caught, exit process...\n");
    rclcpp::shutdown();
}

// Implementation of PlanManager constructor
PlanManager::PlanManager(rclcpp::Node::SharedPtr node, int dof, 
                         std::chrono::seconds joint_tout,
                         std::chrono::seconds pose_tout,
                         int retries)
    : node_(node),
      joint_timeout_(joint_tout),
      pose_timeout_(pose_tout),
      max_retries_(retries),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_)
{
    // Initialize service clients
    joint_plan_client_ = node_->create_client<xarm_msgs::srv::PlanJoint>("xarm_joint_plan");
    pose_plan_client_ = node_->create_client<xarm_msgs::srv::PlanPose>("xarm_pose_plan");
    exec_plan_client_ = node_->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");
    set_scaling_factors_client_ = node_->create_client<xarm_msgs::srv::SetFloat32List>("xarm_set_scaling_factors");
    linear_plan_client_ = node_->create_client<xarm_msgs::srv::PlanSingleStraight>("xarm_straight_plan");
    
    // Initialize the service clients for base and eef links
    get_base_link_client_ = node_->create_client<xarm_msgs::srv::Call>("get_base_link");
    get_eef_link_client_ = node_->create_client<xarm_msgs::srv::Call>("get_eef_link");

    // Initialize joint_names_ordered_ based on dof
    initializeJointNames(dof);

    // Retrieve base and eef links
    if (!retrieveBaseAndEefLinks()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve base and eef links. Exiting.");
        rclcpp::shutdown();
        exit(1);
    }
}

// Implementation of PlanManager::initializeJointNames
void PlanManager::initializeJointNames(int dof) {
    // Define the 7 DOF joint names
    std::vector<std::string> all_joint_names = {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
    };

    if (dof < 5 || dof > 7) {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported DOF: %d. Supported DOFs are 5, 6, and 7.", dof);
        rclcpp::shutdown();
        exit(1);
    }

    // Trim the joint names based on dof
    joint_names_ordered_ = std::vector<std::string>(all_joint_names.begin(), all_joint_names.begin() + dof);
    RCLCPP_INFO(node_->get_logger(), "Initialized joint_names_ordered_ for DOF: %d", dof);
}

// Implementation of PlanManager::retrieveBaseAndEefLinks
bool PlanManager::retrieveBaseAndEefLinks()
{
    // Wait for services to be available
    if (!get_base_link_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Service get_base_link not available.");
        return false;
    }

    if (!get_eef_link_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "Service get_eef_link not available.");
        return false;
    }

    // Call get_base_link service
    auto base_req = std::make_shared<xarm_msgs::srv::Call::Request>();
    auto base_future = get_base_link_client_->async_send_request(base_req);
    if (rclcpp::spin_until_future_complete(node_, base_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service get_base_link");
        return false;
    }
    auto base_res = base_future.get();
    if (base_res->ret != 0) {
        RCLCPP_ERROR(node_->get_logger(), "get_base_link service failed: %s", base_res->message.c_str());
        return false;
    }
    base_link_ = base_res->message;
    RCLCPP_INFO(node_->get_logger(), "Retrieved base link: %s", base_link_.c_str());

    // Call get_eef_link service
    auto eef_req = std::make_shared<xarm_msgs::srv::Call::Request>();
    auto eef_future = get_eef_link_client_->async_send_request(eef_req);
    if (rclcpp::spin_until_future_complete(node_, eef_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service get_eef_link");
        return false;
    }
    auto eef_res = eef_future.get();
    if (eef_res->ret != 0) {
        RCLCPP_ERROR(node_->get_logger(), "get_eef_link service failed: %s", eef_res->message.c_str());
        return false;
    }
    eef_link_ = eef_res->message;
    RCLCPP_INFO(node_->get_logger(), "Retrieved eef link: %s", eef_link_.c_str());

    return true;
}

// Implementation of PlanManager::executeJointMovement
bool PlanManager::executeJointMovement(
    const std::vector<double>& target_joint_positions,
    const std::vector<float>& scaling_factors
) {
    for (int attempt = 0; attempt < max_retries_ && rclcpp::ok(); ++attempt) {

        bool joints_reached = false;
        rclcpp::spin_some(node_);
        joints_reached = checkJointTargets(joint_names_ordered_, target_joint_positions);
        if (joints_reached) {
            RCLCPP_INFO(node_->get_logger(), "Joint targets already reached.");
            return true;
        }

        RCLCPP_INFO(node_->get_logger(), "Attempt %d: Setting scaling factors.", attempt + 1);
        int scaling_result = set_scaling_factors(set_scaling_factors_client_, scaling_factors);
        if (scaling_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Failed to set scaling factors on attempt %d.", attempt + 1);
            // Retry
            continue;
        }

        RCLCPP_INFO(node_->get_logger(), "Attempt %d: Executing joint movement.", attempt + 1);
        auto joint_plan_req = std::make_shared<xarm_msgs::srv::PlanJoint::Request>();
        joint_plan_req->target = target_joint_positions;
        int plan_result = call_request(joint_plan_client_, joint_plan_req);
        if (plan_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Joint plan service call failed on attempt %d.", attempt + 1);
            // Retry
            continue;
        }

        auto exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        exec_plan_req->wait = true;
        int exec_result = call_request(exec_plan_client_, exec_plan_req);
        if (exec_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Joint execution service call failed on attempt %d.", attempt + 1);
            // Retry
            continue;
        }

        // Wait until joint targets are reached or timeout
        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !joints_reached) {
            rclcpp::spin_some(node_);
            joints_reached = checkJointTargets(joint_names_ordered_, target_joint_positions);
            if (joints_reached) {
                break;
            }
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > joint_timeout_) {
                RCLCPP_WARN(node_->get_logger(), "Timeout while waiting for joint targets to be reached on attempt %d.", attempt + 1);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (joints_reached) {
            RCLCPP_INFO(node_->get_logger(), "Joint movement reached on attempt %d.", attempt + 1);
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Joint movement failed on attempt %d.", attempt + 1);
            // Retry after sleep
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    RCLCPP_ERROR(node_->get_logger(), "Failed to execute joint movement after %d attempts.", max_retries_);
    return false;
}

// Implementation of PlanManager::executePoseMovement
bool PlanManager::executePoseMovement(
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<float>& scaling_factors
) {
    for (int attempt = 0; attempt < max_retries_ && rclcpp::ok(); ++attempt) {

        bool pose_reached = false;
        rclcpp::spin_some(node_);
        pose_reached = checkPoseTarget(target_pose, tf_buffer_);
        if (pose_reached) {
            RCLCPP_INFO(node_->get_logger(), "Pose target already reached.");
            return true;
        }

        RCLCPP_INFO(node_->get_logger(), "Attempt %d: Setting scaling factors.", attempt + 1);
        int scaling_result = set_scaling_factors(set_scaling_factors_client_, scaling_factors);
        if (scaling_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Failed to set scaling factors on attempt %d.", attempt + 1);
            // Retry 
            continue;
        }

        RCLCPP_INFO(node_->get_logger(), "Attempt %d: Executing pose movement.", attempt + 1);
        auto pose_plan_req = std::make_shared<xarm_msgs::srv::PlanPose::Request>();
        pose_plan_req->target = target_pose;
        int plan_result = call_request(pose_plan_client_, pose_plan_req);
        if (plan_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Pose plan service call failed on attempt %d.", attempt + 1);
            // Retry 
            continue;
        }

        auto exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        exec_plan_req->wait = true;
        int exec_result = call_request(exec_plan_client_, exec_plan_req);
        if (exec_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Pose execution service call failed on attempt %d.", attempt + 1);
            // Retry 
            continue;
        }

        // Wait until pose target is reached or timeout
        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !pose_reached) {
            rclcpp::spin_some(node_);
            pose_reached = checkPoseTarget(target_pose, tf_buffer_);
            if (pose_reached) {
                break;
            }
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > pose_timeout_) {
                RCLCPP_WARN(node_->get_logger(), "Timeout while waiting for pose target to be reached on attempt %d.", attempt + 1);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (pose_reached) {
            RCLCPP_INFO(node_->get_logger(), "Pose movement reached on attempt %d.", attempt + 1);
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Pose movement failed on attempt %d.", attempt + 1);
            // Retry after sleep
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    RCLCPP_ERROR(node_->get_logger(), "Failed to execute pose movement after %d attempts.", max_retries_);
    return false;
}

// Implementation of PlanManager::executeLinearMovement
bool PlanManager::executeLinearMovement(
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<float>& scaling_factors
) {
    for (int attempt = 0; attempt < max_retries_ && rclcpp::ok(); ++attempt) {

        bool pose_reached = false;
        rclcpp::spin_some(node_);
        pose_reached = checkPoseTarget(target_pose, tf_buffer_);
        if (pose_reached) {
            RCLCPP_INFO(node_->get_logger(), "Linear movement target already reached.");
            return true;
        }

        RCLCPP_INFO(node_->get_logger(), "Attempt %d: Setting scaling factors.", attempt + 1);
        int scaling_result = set_scaling_factors(set_scaling_factors_client_, scaling_factors);
        if (scaling_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Failed to set scaling factors on attempt %d.", attempt + 1);
            // Retry
            continue;
        }

        RCLCPP_INFO(node_->get_logger(), "Attempt %d: Executing linear movement.", attempt + 1);
        auto linear_plan_req = std::make_shared<xarm_msgs::srv::PlanSingleStraight::Request>();
        linear_plan_req->target = target_pose;
        int plan_result = call_request(linear_plan_client_, linear_plan_req);
        if (plan_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Linear plan service call failed on attempt %d.", attempt + 1);
            // Retry
            continue;
        }

        auto exec_plan_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();
        exec_plan_req->wait = true;
        int exec_result = call_request(exec_plan_client_, exec_plan_req);
        if (exec_result != 0) {
            RCLCPP_WARN(node_->get_logger(), "Linear execution service call failed on attempt %d.", attempt + 1);
            // Retry
            continue;
        }

        // Wait until pose target is reached or timeout
        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !pose_reached) {
            rclcpp::spin_some(node_);
            pose_reached = checkPoseTarget(target_pose, tf_buffer_);
            if (pose_reached) {
                break;
            }
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed > pose_timeout_) {
                RCLCPP_WARN(node_->get_logger(), "Timeout while waiting for linear movement to be reached on attempt %d.", attempt + 1);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (pose_reached) {
            RCLCPP_INFO(node_->get_logger(), "Linear movement reached on attempt %d.", attempt + 1);
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Linear movement failed on attempt %d.", attempt + 1);
            // Retry after sleep
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    RCLCPP_ERROR(node_->get_logger(), "Failed to execute linear movement after %d attempts.", max_retries_);
    return false;
}

// Implementation of PlanManager::getJointNames
const std::vector<std::string>& PlanManager::getJointNames() const {
    return joint_names_ordered_;
}
