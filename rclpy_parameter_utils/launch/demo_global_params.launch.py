from os import path

import yaml

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration

def print_instructions():
    print('\n------------------INSTRUCTIONS -------------------------')
    print('1 - See the parameter list using:\n\t\t"ros2 param list"')
    print('2 - Get a parameter using the command:\n\t\t"ros2 param get <server_name> <param_name>"')
    print('3 - Then run "ros2 run rclpy_parameter_utils demo_global_params_client.py" and query the parameters to see the new values')
    print('\n')

def load_yaml(yaml_file_path):
    with open(yaml_file_path,'r') as f:
        return yaml.load(f)

def launch_setup(context, *args, **kwargs):
    
    parameter_server_name = LaunchConfiguration('parameter_server_name').perform(context)
    parameter_yaml_file  = LaunchConfiguration('param_yaml_file').perform(context)
    parameters_yaml = load_yaml(parameter_yaml_file)
    
    print(str(parameters_yaml))
    
    
    parameter_server_node = Node(
        executable = 'parameters_action_server.py',
        package = 'rclpy_parameter_utils',
        name = parameter_server_name,
        output = 'screen',
        parameters = [parameters_yaml],
        respawn_delay = 5.0,
        arguments = '--ros-args --log-level info'.split())
    
    example_parameter_client_node = Node(
        executable = 'demo_parameters_client.py',
        package = 'rclpy_parameter_utils',
        name = 'example_parameter_client',
        parameters = [{'parameter_server_name': parameter_server_name}],
        output = 'screen')
    
    nodes_list = [parameter_server_node]
    #nodes_list.append(example_parameter_client_node)
    nodes_list.append(OpaqueFunction(function = lambda context, *args, **kwargs : print_instructions()))
        
    return nodes_list

def generate_launch_description():
    
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument('param_yaml_file',
        default_value = path.join(get_package_share_directory('rclpy_parameter_utils'), 'launch','example_params.yaml')))
    
    ld.add_action(DeclareLaunchArgument('parameter_server_name', default_value = 'global_parameter_server'))
    
    ld.add_action(OpaqueFunction(function = launch_setup))
    
    return ld
    
    