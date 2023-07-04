from launch import LaunchDescription, conditions
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, Command

import launch.actions
import launch_ros.actions

def generate_launch_description():
	
	return LaunchDescription([
	DeclareLaunchArgument(
        'skeleton_frame',
        default_value="depth_camera_link",
        description="Used by recorder for bags. Specify the frame to be used for the recorded body tracking joints"),
	DeclareLaunchArgument(
        'prediction_threshold',
        default_value="0.3",
        description="Threshold for prediction"),
	DeclareLaunchArgument(
        'skeleton_to_rgb_enabled',
        default_value="true",
        description="Enable or disable the rgb image with the skeleton overlay"),
    DeclareLaunchArgument(
        'plot_model_output',
        default_value="false",
        description="If True, it does not plot the skeletons, but a bounding box with model output"),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='skeleton_to_rgb.py',
        name='skeleton_to_rgb_node',
        parameters = [{'plot_model_output': launch.substitutions.LaunchConfiguration('plot_model_output')}],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("skeleton_to_rgb_enabled"))),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='model_node.py',
        name='skeleton_to_rgb_node',
        parameters = [],
        ),
    ])