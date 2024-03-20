import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    toolbox_dir=get_package_share_directory('multi-laserscan-toolbox-ros2')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(toolbox_dir, 'config', 'laserscan_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    node=Node(
        parameters=[
          params_file,
          {'use_sim_time': use_sim_time}
        ],
        package = 'multi-laserscan-toolbox-ros2',
        name = 'laserscan_toolbox_node',
        executable = 'laserscan_toolbox_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(node)
    return ld