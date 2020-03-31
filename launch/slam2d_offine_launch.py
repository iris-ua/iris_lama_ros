from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch.actions
import pathlib

def generate_launch_description():
    # https://answers.ros.org/question/311456/how-to-launch-a-node-with-a-parameter-in-ros2/
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) + '/config/offline_mode.yaml'
    print(parameters_file_path)

    return LaunchDescription([
        launch_ros.actions.DeclareLaunchArgument('scan_topic',     default_value="/scan", description=''),
        launch_ros.actions.DeclareLaunchArgument('rosbag',         default_value="", description=''),

        launch_ros.actions.DeclareLaunchArgument('particles',      default_value=30, description=''),
        launch_ros.actions.DeclareLaunchArgument('d_thresh',       default_value=0.5, description=''),
        launch_ros.actions.DeclareLaunchArgument('a_thresh',       default_value=0.5, description=''),
        launch_ros.actions.DeclareLaunchArgument('mrange',         default_value=20, description=''),

        launch_ros.actions.DeclareLaunchArgument('threads',        default_value=4, description=''),
        launch_ros.actions.DeclareLaunchArgument('use_compression',default_value=False, description=''),

        launch_ros.actions.DeclareLaunchArgument('/use_sim_time',  default_value=True, description=''),
        Node(
            package='iris_lama_ros',
            node_namespace='iris_lama_ros',
            node_executable='slam2d_ros',
            node_name='slam2d_ros',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel')
            ],
            output='screen',
            parameters=[parameters_file_path],
        )
    ])