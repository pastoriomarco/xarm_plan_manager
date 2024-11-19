from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dof = LaunchConfiguration('dof', default=6)

    xarm_planner_node_test = Node(
        #name='plan_manager_node',
        package='xarm_plan_manager',
        executable='plan_manager_node',
        output='screen',
        parameters=[
            {
                'dof': dof
            },
        ],
    )
    return LaunchDescription([
        xarm_planner_node_test
    ])
