from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
import launch
import launch.actions
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
import pathlib

def generate_launch_description():
    # https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/
    # https://answers.ros.org/question/311456/how-to-launch-a-node-with-a-parameter-in-ros2/
    # https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) + '/config/live.yaml'
    print(parameters_file_path)

    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='false')
    use_composition = LaunchConfiguration('use_composition')

    return LaunchDescription([
        #launch.actions.DeclareLaunchArgument('particles',      default_value="30", description=''),
        #launch.actions.DeclareLaunchArgument('threads',        default_value="4", description=''),
        #launch.actions.DeclareLaunchArgument('/use_sim_time',  default_value="True", description=''),
        declare_use_composition_cmd,
        Node(
            package='iris_lama_ros2',
            namespace='iris_lama_ros2',
            executable='pf_slam2d_ros',
            name='pf_slam2d_ros',
            #remappings=[
            #    ('/input/pose', '/turtlesim1/turtle1/pose'),
            #    ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel')
            #],
            output='screen',
            parameters=[parameters_file_path],
            condition=UnlessCondition(use_composition)
        ),
        ComposableNodeContainer(
            name='iris_lama_container', 
            package='rclcpp_components',
            executable='component_container',
            namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='iris_lama_ros2',
                    plugin='lama::PFSlam2DROS',
                    name='pf_slam2d_ros',
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
            parameters=[parameters_file_path],
            condition=IfCondition(use_composition)
        ),
    ])