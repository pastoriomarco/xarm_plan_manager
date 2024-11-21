// src/xarm_plan_manager/src/plan_manager_node.cpp

#include "xarm_plan_manager/plan_manager.hpp"

// Joint state callback function to parse the content of /joint_states topic
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg, sensor_msgs::msg::JointState &current_joint_state_)
{
    // Copy joint state data to the current_joint_state_ variable
    current_joint_state_ = *msg;
}

// Implementation of main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("plan_manager_node", node_options);
    RCLCPP_INFO(node->get_logger(), "plan_manager_node start");

    int dof;
    node->get_parameter_or("dof", dof, 7);
    RCLCPP_INFO(node->get_logger(), "namespace: %s, dof: %d", node->get_namespace(), dof);

    // Read the 'robot_type' parameter
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    RCLCPP_INFO(node->get_logger(), "Robot Type: %s", robot_type.c_str());

    // Validate 'robot_type' based on 'dof'
    if (dof == 5 || dof == 7)
    {
        if (robot_type != "xarm")
        {
            RCLCPP_ERROR(node->get_logger(), "Unsupported robot_type '%s' for DOF %d. Only 'xarm' is supported for DOF 5 and 7.", robot_type.c_str(), dof);
            rclcpp::shutdown();
            return 1;
        }
    }
    else if (dof == 6)
    {
        if (robot_type != "xarm" && robot_type != "lite" && robot_type != "uf850")
        {
            RCLCPP_ERROR(node->get_logger(), "Unsupported robot_type '%s' for DOF %d. Supported types: 'xarm', 'lite', 'uf850'.", robot_type.c_str(), dof);
            rclcpp::shutdown();
            return 1;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Unsupported DOF: %d. Supported DOFs are 5, 6, and 7. Exiting.", dof);
        rclcpp::shutdown();
        return 1;
    }

    // Initialize PlanManager with the DOF
    PlanManager plan_manager(node, dof, std::chrono::seconds(30), std::chrono::seconds(30), 3);

    // Local variable for joint states
    sensor_msgs::msg::JointState current_joint_state_;

    // Subscribe to /joint_states
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [&current_joint_state_](const sensor_msgs::msg::JointState::SharedPtr msg)
        { jointStateCallback(msg, current_joint_state_); });

    // Parameter to determine which object to find
    std::string object_name_filter;
    node->declare_parameter<std::string>("object_name_filter", "graspable_cylinder_spawner");
    node->get_parameter("object_name_filter", object_name_filter);

    // Service client to get the planning scene
    auto planning_scene_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    while (!planning_scene_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for the /get_planning_scene service to be available...");
    }

    // Get the planning scene
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    geometry_msgs::msg::Pose selected_object_pose;
    std::string selected_object_id;
    bool object_found = false;
    auto future = planning_scene_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        const auto &collision_objects = response->scene.world.collision_objects;

        for (const auto &object : collision_objects)
        {
            if (object.id.find(object_name_filter) != std::string::npos)
            {
                // Use the main pose field instead of primitive_poses to get the correct pose
                selected_object_pose = object.pose;
                selected_object_id = object.id;
                RCLCPP_INFO(node->get_logger(), "Found object with ID: %s", selected_object_id.c_str());
                RCLCPP_INFO_STREAM(node->get_logger(), "x " << selected_object_pose.position.x << ", y " << selected_object_pose.position.y << ", z " << selected_object_pose.position.z);
                object_found = true;
                break;
            }
        }

        if (!object_found)
        {
            RCLCPP_WARN(node->get_logger(), "No object found with filter: %s", object_name_filter.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service /get_planning_scene");
    }

    // Wait until the first joint state is received
    RCLCPP_INFO(node->get_logger(), "Waiting for first joint state...");
    while (rclcpp::ok() && current_joint_state_.name.empty())
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node->get_logger(), "Received first joint state. Starting movements.");

    // Create a map for joint positions by name
    std::unordered_map<std::string, double> joint_positions;
    for (size_t i = 0; i < current_joint_state_.name.size(); ++i)
    {
        joint_positions[current_joint_state_.name[i]] = current_joint_state_.position[i];
    }

    // Define target joint positions based on DOF and robot_type
    std::vector<double> tar_joint1;
    std::vector<double> tar_joint2;
    std::vector<double> tar_joint3;
    std::vector<double> tar_joint4;

    switch (dof)
    {
    case 5:
    {
        // Only 'xarm' is supported for DOF 5
        tar_joint1 = {joint_positions["joint1"], -1.570796, -1.047198, 2.792527, -1.570796};
        tar_joint2 = {0, 0, 0, 0, 0};
        tar_joint3 = {-1.570796, -1.570796, -1.047198, -0.349066, 2.617994};
    }
    break;
    case 6:
    {
        if ((robot_type == "xarm") || (robot_type == "uf850"))
        {
            tar_joint1 = {0.785, -0.0, -1.75, -0.785, 0.785, -0.785};
            tar_joint2 = {0, 0, 0, 0, 0, 0};
            tar_joint3 = {-0.785, -0.785, -0.75, 0.785, -0.785, 0.785};
            tar_joint4 = {0, 0, -1.57, 0, 0, 0};
        }
        else if (robot_type == "lite")
        {
            // Define target joints for 'lite' 6 DOF
            tar_joint1 = {0.785, -0.0, 1.75, -0.785, 0.785, -0.785};
            tar_joint2 = {0, 0, 0, 0, 0, 0};
            tar_joint3 = {-0.785, -0.785, 0.75, 0.785, -0.785, 0.785};
            tar_joint4 = {0, 0, 1.57, 0, 0, 0};
        }
    }
    break;
    case 7:
    {
        // Only 'xarm' is supported for DOF 7
        tar_joint1 = {1.570796, -1.570796, -1.570796, 1.396263, 2.967060, 2.792527, -1.570796};
        tar_joint2 = {0, 0, 0, 0, 0, 0, 0};
        tar_joint3 = {-1.570796, -1.570796, 1.570796, 1.396263, -2.967060, -0.349066, 2.617994};
    }
    break;
    default:
    {
        RCLCPP_ERROR(node->get_logger(), "Unsupported DOF: %d. Supported DOFs are 5, 6, and 7. Exiting.", dof);
        rclcpp::shutdown();
        return 1;
    }
    break;
    }

    // Define target poses
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = -0.15;
    target_pose1.position.z = 0.1;
    target_pose1.orientation.x = 1;
    target_pose1.orientation.y = 0;
    target_pose1.orientation.z = 0;
    target_pose1.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = 0.3;
    target_pose2.position.y = 0.1;
    target_pose2.position.z = 0.2;
    target_pose2.orientation.x = 1;
    target_pose2.orientation.y = 0;
    target_pose2.orientation.z = 0;
    target_pose2.orientation.w = 0;

    geometry_msgs::msg::Pose target_pose3;
    target_pose3.position.x = 0.1;
    target_pose3.position.y = 0.25;
    target_pose3.position.z = 0.15;
    target_pose3.orientation.x = 1;
    target_pose3.orientation.y = 0;
    target_pose3.orientation.z = 0;
    target_pose3.orientation.w = 0;

    selected_object_pose.orientation = target_pose1.orientation;
    geometry_msgs::msg::Pose selected_object_approach_pose = selected_object_pose;
    selected_object_approach_pose.position.z = selected_object_pose.position.z + 0.1;

    // Example scaling factors to be set between movements
    std::vector<float> slow_speed = {0.1f, 0.1f};     // Example values
    std::vector<float> standard_speed = {0.3f, 0.3f}; // Example values
    std::vector<float> high_speed = {0.5f, 0.5f};     // Example values

    // Execute movements with retry logic using PlanManager
    if (!plan_manager.executeJointMovement(tar_joint2, high_speed))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to execute tar_joint2 after retries. Exiting.");
        rclcpp::shutdown();
        return 1;
    }

    if (object_found)
    {
        // 2. object grasp with standard_speed
        if (!plan_manager.executePoseMovement(selected_object_approach_pose, standard_speed))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute selected_object_approach_pose after retries. Exiting.");
            rclcpp::shutdown();
            return 1;
        }

        if (!plan_manager.executePoseMovement(selected_object_pose, standard_speed))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute selected_object_pose after retries. Exiting.");
            rclcpp::shutdown();
            return 1;
        }

        // After moving to selected_object_pose
        if (!plan_manager.attachObject(selected_object_id))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to attach object '%s'. Exiting.", selected_object_id.c_str());
            rclcpp::shutdown();
            return 1;
        }

        if (!plan_manager.executePoseMovement(selected_object_approach_pose, standard_speed))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute selected_object_approach_pose after retries. Exiting.");
            rclcpp::shutdown();
            return 1;
        }
    }

    // target_pose1 with standard_speed
    if (!plan_manager.executePoseMovement(target_pose1, slow_speed))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to execute target_pose4 linear move after retries. Exiting.");
        rclcpp::shutdown();
        return 1;
    }

    if (object_found)
    { // After moving to target_pose2
        if (!plan_manager.detachObject(selected_object_id))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to detach object '%s'. Exiting.", selected_object_id.c_str());
            rclcpp::shutdown();
            return 1;
        }

        // exit move from target_pose1 with standard_speed
        geometry_msgs::msg::Pose exit_target_pose1 = target_pose1;
        exit_target_pose1.position.z = exit_target_pose1.position.z + 0.1;
        if (!plan_manager.executePoseMovement(exit_target_pose1, slow_speed))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to execute target_pose4 linear move after retries. Exiting.");
            rclcpp::shutdown();
            return 1;
        }
    }

    // tar_joint4 with high_speed
    if (!plan_manager.executeJointMovement(tar_joint4, high_speed))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to execute tar_joint4 after retries. Exiting.");
        rclcpp::shutdown();
        return 1;
    }

    // // 1. tar_joint1 with slow_speed
    // if (!plan_manager.executeJointMovement(tar_joint1, slow_speed))
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to execute tar_joint1 after retries. Exiting.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // target_pose2 with standard_speed
    // if (!plan_manager.executePoseMovement(target_pose2, standard_speed))
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to execute target_pose2 after retries. Exiting.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // tar_joint3 with slow_speed
    // if (!plan_manager.executeJointMovement(tar_joint3, slow_speed))
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to execute tar_joint3 after retries. Exiting.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // target_pose3 with standard_speed
    // if (!plan_manager.executePoseMovement(target_pose3, standard_speed))
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to execute target_pose3 after retries. Exiting.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // not using linear moves: the robot seems to be moving at max speed despite setting the scaling factors
    // // // target_pose4 with standard_speed - linear move
    // // if (!plan_manager.executeLinearMovement(target_pose2, slow_speed)) {
    // //     RCLCPP_ERROR(node->get_logger(), "Failed to execute target_pose4 linear move after retries. Exiting.");
    // //     rclcpp::shutdown();
    // //     return 1;
    // // }

    RCLCPP_INFO(node->get_logger(), "Completed all movements successfully.");
    rclcpp::shutdown();
    return 0;
}
