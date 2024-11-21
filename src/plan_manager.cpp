#include "xarm_plan_manager/plan_manager.hpp"

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

    // Initialize the new service clients
    get_planning_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_planning_scene_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

    // Initialize joint_names_ordered_ based on dof
    initializeJointNames(dof);

    // Retrieve base and eef links
    if (!retrieveBaseAndEefLinks()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve base and eef links. Exiting.");
        rclcpp::shutdown();
        exit(1);
    }

    // Initialize the joint state subscriber
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&PlanManager::jointStateCallback, this, std::placeholders::_1)
    );
}

// Destructor
PlanManager::~PlanManager()
{
    // Shutdown the node if necessary
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

// Implementation of jointStateCallback
void PlanManager::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    current_joint_state_ = msg;
}

// Implementation of checkJointTargets
bool PlanManager::checkJointTargets(const std::vector<std::string>& joint_names_ordered,
                                    const std::vector<double>& target_joint_positions,
                                    double tolerance)
{
    if (!current_joint_state_) {
        RCLCPP_WARN(node_->get_logger(), "No joint state received yet.");
        return false;
    }

    // Create a map from joint name to position for quick lookup
    std::unordered_map<std::string, double> current_joints_map;
    for (size_t i = 0; i < current_joint_state_->name.size(); ++i) {
        current_joints_map[current_joint_state_->name[i]] = current_joint_state_->position[i];
    }

    // Iterate over the ordered joint names and compare positions
    for (size_t i = 0; i < joint_names_ordered.size(); ++i) {
        const std::string& joint_name = joint_names_ordered[i];
        if (current_joints_map.find(joint_name) == current_joints_map.end()) {
            RCLCPP_WARN(node_->get_logger(), "Joint name %s not found in current joint states.", joint_name.c_str());
            return false;
        }
        double current = current_joints_map[joint_name];
        double target = target_joint_positions[i];
        if (std::abs(current - target) > tolerance) {
            // For debugging: Log which joint is not within tolerance
            RCLCPP_DEBUG(node_->get_logger(), "Joint %s: current=%.4f, target=%.4f", joint_name.c_str(), current, target);
            return false;
        }
    }
    return true;
}

// Implementation of checkPoseTarget
bool PlanManager::checkPoseTarget(const geometry_msgs::msg::Pose& target_pose,
                                  double position_tolerance,
                                  double orientation_tolerance)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        // Use dynamic base and eef links
        transformStamped = tf_buffer_.lookupTransform(base_link_, eef_link_, tf2::TimePointZero, std::chrono::seconds(1));
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
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
        RCLCPP_DEBUG(node_->get_logger(), "Pose position difference exceeds tolerance.");
        return false;
    }

    // Compare orientations (quaternions)
    if (angle_diff > orientation_tolerance) {
        RCLCPP_DEBUG(node_->get_logger(), "Pose orientation difference exceeds tolerance.");
        return false;
    }

    return true;
}

// Implementation of retrieveBaseAndEefLinks
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

// Implementation of initializeJointNames
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

// Implementation of set_scaling_factors
int PlanManager::set_scaling_factors(rclcpp::Client<xarm_msgs::srv::SetFloat32List>::SharedPtr client, const std::vector<float>& datas)
{
    auto request = std::make_shared<xarm_msgs::srv::SetFloat32List::Request>();
    request->datas = datas;

    // Use the existing call_request function
    int result = call_request(client, request);
    return result;
}

// Implementation of executeJointMovement
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

// Implementation of executePoseMovement
bool PlanManager::executePoseMovement(
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<float>& scaling_factors
) {
    for (int attempt = 0; attempt < max_retries_ && rclcpp::ok(); ++attempt) {

        bool pose_reached = false;
        rclcpp::spin_some(node_);
        pose_reached = checkPoseTarget(target_pose);
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
            pose_reached = checkPoseTarget(target_pose);
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

// Implementation of executeLinearMovement
bool PlanManager::executeLinearMovement(
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<float>& scaling_factors
) {
    for (int attempt = 0; attempt < max_retries_ && rclcpp::ok(); ++attempt) {

        bool pose_reached = false;
        rclcpp::spin_some(node_);
        pose_reached = checkPoseTarget(target_pose);
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
            pose_reached = checkPoseTarget(target_pose);
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

// Implementation of attachObject
bool PlanManager::attachObject(const std::string& object_id) {
    // Get the collision object from the planning scene
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    // Wait for the service and call it
    while (!get_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service get_planning_scene. Exiting.");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for service get_planning_scene...");
    }

    auto future = get_planning_scene_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service get_planning_scene");
        return false;
    }

    auto response = future.get();
    const auto& collision_objects = response->scene.world.collision_objects;

    moveit_msgs::msg::CollisionObject object_to_attach;
    bool object_found = false;

    for (const auto& object : collision_objects) {
        if (object.id == object_id) {
            object_to_attach = object;
            object_found = true;
            break;
        }
    }

    if (!object_found) {
        RCLCPP_ERROR(node_->get_logger(), "Object with id '%s' not found in planning scene.", object_id.c_str());
        return false;
    }

    // Create AttachedCollisionObject
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = eef_link_;
    attached_object.object = object_to_attach;
    attached_object.touch_links = std::vector<std::string>({eef_link_});

    // Prepare PlanningScene message
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.world.collision_objects.clear(); // Remove the object from world

    // Send the PlanningScene message
    auto apply_planning_scene_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_planning_scene_request->scene = planning_scene;

    while (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service apply_planning_scene. Exiting.");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for service apply_planning_scene...");
    }

    auto apply_future = apply_planning_scene_client_->async_send_request(apply_planning_scene_request);
    if (rclcpp::spin_until_future_complete(node_, apply_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service apply_planning_scene");
        return false;
    }

    auto apply_response = apply_future.get();
    if (!apply_response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to apply planning scene to attach object.");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Object '%s' successfully attached to '%s'", object_id.c_str(), eef_link_.c_str());
    return true;
}

// Implementation of detachObject
bool PlanManager::detachObject(const std::string& object_id) {
    // 1. Get the collision object from the planning scene
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

    // Wait for the service and call it
    while (!get_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service get_planning_scene. Exiting.");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for service get_planning_scene...");
    }

    auto future = get_planning_scene_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service get_planning_scene");
        return false;
    }

    auto response = future.get();
    const auto& attached_objects = response->scene.robot_state.attached_collision_objects;

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    bool object_found = false;
    for (const auto& attached : attached_objects) {
        if (attached.object.id == object_id) {
            attached_object = attached;
            object_found = true;
            break;
        }
    }

    if (!object_found) {
        RCLCPP_ERROR(node_->get_logger(), "Object with ID '%s' is not attached to the robot.", object_id.c_str());
        return false;
    }

    // 2. Get the current transform of the object relative to the base frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform(base_link_, attached_object.link_name, tf2::TimePointZero, std::chrono::seconds(1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }

    // Convert the TransformStamped to tf2::Transform
    tf2::Transform base_to_link;
    tf2::fromMsg(transform_stamped.transform, base_to_link);

    // Get the pose of the object relative to the link
    geometry_msgs::msg::Pose object_pose_in_link = attached_object.object.pose;

    // Convert object_pose_in_link to tf2::Transform
    tf2::Transform link_to_object;
    tf2::fromMsg(object_pose_in_link, link_to_object);

    // Compute base_to_object = base_to_link * link_to_object
    tf2::Transform base_to_object = base_to_link * link_to_object;

    // Manually construct geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose object_pose;
    object_pose.position.x = base_to_object.getOrigin().x();
    object_pose.position.y = base_to_object.getOrigin().y();
    object_pose.position.z = base_to_object.getOrigin().z();
    object_pose.orientation.x = base_to_object.getRotation().x();
    object_pose.orientation.y = base_to_object.getRotation().y();
    object_pose.orientation.z = base_to_object.getRotation().z();
    object_pose.orientation.w = base_to_object.getRotation().w();

    // 3. Detach the Object from the End Effector
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = object_id;
    detach_object.link_name = attached_object.link_name;
    detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    // 4. Create a Collision Object to Add Back to the World
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = attached_object.object.id;
    collision_object.header.frame_id = base_link_;
    collision_object.pose = object_pose;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_object.primitives = attached_object.object.primitives;
    collision_object.primitive_poses = attached_object.object.primitive_poses;
    collision_object.meshes = attached_object.object.meshes;
    collision_object.mesh_poses = attached_object.object.mesh_poses;
    collision_object.planes = attached_object.object.planes;
    collision_object.plane_poses = attached_object.object.plane_poses;

    // 5. Prepare the Planning Scene Message
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.robot_state.is_diff = true;
    planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene.world.collision_objects.push_back(collision_object);

    // 6. Apply the Planning Scene
    auto apply_planning_scene_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_planning_scene_request->scene = planning_scene;

    while (!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the apply_planning_scene service. Exiting.");
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for apply_planning_scene service...");
    }

    auto apply_future = apply_planning_scene_client_->async_send_request(apply_planning_scene_request);
    if (rclcpp::spin_until_future_complete(node_, apply_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service apply_planning_scene");
        return false;
    }

    auto apply_response = apply_future.get();
    if (!apply_response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to apply planning scene to detach object.");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Object '%s' successfully detached and added back to world.", object_id.c_str());
    return true;
}
