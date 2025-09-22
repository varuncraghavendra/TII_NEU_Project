
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def run_launch_arguments():
    args = [
        # Lifecycle arguments
        DeclareLaunchArgument(
            'auto_start', default_value='true', choices=['true','false'],
            description='Auto-start lifecycle nodes.'
        )
    ]
    return args


def run_lifecycle_actions():
    def _setup(context, *args, **kwargs):
        bt_nodes ="drone_bt_navigator"
        fc_nodes ="flight_control_node"
        life_nodes =  [] + [bt_nodes] + [fc_nodes] 
        log_action = LogInfo(msg=f"Unified Lifecycle Manager will manage: {life_nodes}")
        manager_action = create_unified_lifecycle_manager(bt_nodes, fc_nodes)
        return [log_action, manager_action]

    return [OpaqueFunction(function=_setup)]

def create_unified_lifecycle_manager(*args: str):
    node_list = []
    for arg in args:
        if not isinstance(arg, str):
            raise TypeError(f"All arguments must be strings. Got {type(arg).__name__}")
        else:
            node_list.append(arg)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='unified_lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': LaunchConfiguration('auto_start')},
            {'node_names': node_list},
            {'bond_timeout': 0.0}
        ],
        condition=IfCondition(LaunchConfiguration('auto_start'))
    )
    return lifecycle_manager_node

def launcher_flightcontrol_node_manager(pkg_share_dir):
    fc_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch_flightctrl_node.launch.py')
        )
    )
    return [fc_include]

def launcher_bt_navigator_manager(pkg_share_dir):
    bt_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch_bt_node.launch.py')
        )
    )
    return [bt_include]

def generate_launch_description():
    share_path_flight_control_node_manager = get_package_share_directory('flight_control')
    share_path_bt_navigator = get_package_share_directory('bt_navigator')
    
    fligth_control_manager   = launcher_flightcontrol_node_manager(share_path_flight_control_node_manager)
    bt_navigator_manager   = launcher_bt_navigator_manager(share_path_bt_navigator)

    lifecycle_actions = run_lifecycle_actions()
    args = run_launch_arguments()
    
    gm_node = Node(
        package="goal_manager",
        executable="goal_manager_node",
    )
    return LaunchDescription(
        args + fligth_control_manager + bt_navigator_manager + lifecycle_actions + [gm_node]
    )
