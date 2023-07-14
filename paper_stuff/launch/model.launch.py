from launch import LaunchDescription, conditions
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, Command

import launch.actions
import launch_ros.actions

def generate_launch_description():
	
	return LaunchDescription([
	DeclareLaunchArgument(
        'skeleton_frame',
        default_value="camera_base",
        description="Specify the frame to be used for the model input"),
	DeclareLaunchArgument(
        'prediction_threshold',
        default_value="0.92",
        description="Threshold for prediction"),
    DeclareLaunchArgument(
        'hysteresis',
        default_value="True",
        description="If True, prediction_threshold is upper threshold and hysteresis_threshold is lower threshold for hysteresis"),
    DeclareLaunchArgument(
        'hysteresis_threshold',
        default_value="0.3",
        description="Lower threshold for hysteresis"),
    DeclareLaunchArgument(
        'user_alive_time',
        default_value="5.",
        description="Drop user prediction after this time without tracking. Used to reset hysteresis."),
	DeclareLaunchArgument(
        'skeleton_to_rgb_enabled',
        default_value="true",
        description="Enable or disable the rgb image with the skeleton overlay"),
    DeclareLaunchArgument(
        'plot_model_output',
        default_value="true",
        description="If True, it does not plot the skeletons, but a bounding box with model output"),
    DeclareLaunchArgument(
        'lstm',
        default_value="true",
        description="If True, it does not plot the skeletons, but a bounding box with model output"),
    launch_ros.actions.Node(
        package='paper_stuff',
        executable='skeleton_to_rgb',
        name='skeleton_to_rgb_node',
        parameters = [{'plot_model_output': launch.substitutions.LaunchConfiguration('plot_model_output')},
                      {'user_alive_time': launch.substitutions.LaunchConfiguration('user_alive_time')},
                      {'hysteresis': launch.substitutions.LaunchConfiguration('hysteresis')},
                      {'prediction_threshold': launch.substitutions.LaunchConfiguration('prediction_threshold')},
                      {'hysteresis_threshold': launch.substitutions.LaunchConfiguration('hysteresis_threshold')}],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("skeleton_to_rgb_enabled"))),
    launch_ros.actions.Node(
        package='paper_stuff',
        executable='lstm_node',
        name='lstm_node',
        parameters = [{'skeleton_frame': launch.substitutions.LaunchConfiguration('skeleton_frame')},
                      {'prediction_threshold': launch.substitutions.LaunchConfiguration('prediction_threshold')}],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("lstm"))
        ),
    launch_ros.actions.Node(
        package='paper_stuff',
        executable='rf_node',
        name='rf_node',
        parameters = [{'skeleton_frame': launch.substitutions.LaunchConfiguration('skeleton_frame')},
                      {'prediction_threshold': launch.substitutions.LaunchConfiguration('prediction_threshold')}],
        condition=conditions.UnlessCondition(launch.substitutions.LaunchConfiguration("lstm"))
        ),
    ])