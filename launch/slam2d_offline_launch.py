from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch.actions
import pathlib

def generate_launch_description():
    # https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/
    # https://answers.ros.org/question/311456/how-to-launch-a-node-with-a-parameter-in-ros2/
    # https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) + '/config/offline_mode.yaml'
    print(parameters_file_path)

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('scan_topic',     default_value="/scan", description=''),
        launch.actions.DeclareLaunchArgument('rosbag',         default_value="", description=''),

        launch.actions.DeclareLaunchArgument('particles',      default_value="30", description=''),
        launch.actions.DeclareLaunchArgument('d_thresh',       default_value="0.5", description=''),
        launch.actions.DeclareLaunchArgument('a_thresh',       default_value="0.5", description=''),
        launch.actions.DeclareLaunchArgument('mrange',         default_value="20", description=''),

        launch.actions.DeclareLaunchArgument('threads',        default_value="4", description=''),
        launch.actions.DeclareLaunchArgument('use_compression',default_value="False", description=''),

        launch.actions.DeclareLaunchArgument('/use_sim_time',  default_value="True", description=''),
        Node(
            package='iris_lama_ros2',
            node_namespace='iris_lama_ros2',
            node_executable='slam2d_ros',
            node_name='slam2d_ros',
            #remappings=[
            #    ('/input/pose', '/turtlesim1/turtle1/pose'),
            #    ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel')
            #],
            output='screen',
            parameters=[parameters_file_path],
        )
    ])