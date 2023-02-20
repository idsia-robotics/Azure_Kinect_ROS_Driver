# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, Command

import launch.actions
import launch_ros.actions

def to_urdf(xacro_path, urdf_path=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * urdf_path -- the path to the urdf file
    """
    # If no URDF path is given, use a temporary file
    if urdf_path is None:
        urdf_path = os.path.join(
            get_package_share_directory("azure_kinect_ros_driver"),
            "urdf",
            "azure_kinect.urdf")
    # open and process file
    doc = xacro.process_file(xacro_path)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path  # Return path to the urdf file

def generate_launch_description():
    # Note: tf_prefix is not supported as an argument to the xacro file for robot/joint state publishers
    # Convert xacro to urdf for robot_state_publisher and joint_state_publisher
    xacro_file = os.path.join(
            get_package_share_directory("azure_kinect_ros_driver"),
            "urdf",
            "azure_kinect.urdf.xacro")
    print("Robot description xacro_file : {}".format(xacro_file))

    urdf_path = to_urdf(xacro_file) # convert, xacro to urdf
    urdf = open(urdf_path).read()
    print("Robot description urdf_path : {}".format(urdf_path))

    # Variable used for the flag to publish a standalone azure_description instead of the default robot_description parameter
    remappings = [('robot_description', 'azure_description')]
    use_feedback_node = launch.substitutions.PythonExpression(
        [launch.substitutions.LaunchConfiguration('audio_feedback'), ' or ', launch.substitutions.LaunchConfiguration('telegram_feedback')]
        )
    
    return LaunchDescription([
    DeclareLaunchArgument(
        'overwrite_robot_description',
        default_value="true" ,
        description="Flag to publish a standalone azure_description instead of the default robot_description parameter."),
    ##############################################
    DeclareLaunchArgument(
        'depth_enabled',
        default_value="true",
        description="Enable or disable the depth camera"),
    DeclareLaunchArgument(
        'depth_mode',
        default_value="WFOV_UNBINNED",
        description="Set the depth camera mode, which affects FOV, depth range, and camera resolution. See Azure Kinect documentation for full details. Valid options: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR"),
    DeclareLaunchArgument(
        'depth_unit',
        default_value="16UC1",
        description='Depth distance units. Options are: "32FC1" (32 bit float metre) or "16UC1" (16 bit integer millimetre)'),
    DeclareLaunchArgument(
        'color_enabled',
        default_value="true",
        description="Enable or disable the color camera"),
    DeclareLaunchArgument(
        'color_format',
        default_value="bgra",
        description="The format of RGB camera. Valid options: bgra, jpeg"),
    DeclareLaunchArgument(
        'color_resolution',
        default_value="1536P",
        description="Resolution at which to run the color camera. Valid options: 720P, 1080P, 1440P, 1536P, 2160P, 3072P"),
    DeclareLaunchArgument(
        'microphone_enabled',
        default_value="true",
        description="Enable or disable the 7 channel microphone"),
    DeclareLaunchArgument(
        'fps',
        default_value="5",
        description="FPS to run both cameras at. Valid options are 5, 15, and 30"),
    DeclareLaunchArgument(
        'point_cloud',
        default_value="false",
        description="Generate a point cloud from depth data. Requires depth_enabled"),
    DeclareLaunchArgument(
        'rgb_point_cloud',
        default_value="false",
        description="Colorize the point cloud using the RBG camera. Requires color_enabled and depth_enabled"),
    DeclareLaunchArgument(
        'point_cloud_in_depth_frame',
        default_value="false",
        description="Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). Will either match the resolution of the depth camera (true) or the RGB camera (false)."),
    DeclareLaunchArgument( # Not a parameter of the node, rather a launch file parameter
        'required',
        default_value="false",
        description="Argument which specified if the entire launch file should terminate if the node dies"),
    DeclareLaunchArgument(
        'sensor_sn',
        default_value="",
        description="Sensor serial number. If none provided, the first sensor will be selected"),
    DeclareLaunchArgument(
        'recording_file',
        default_value="",
        description="Absolute path to a mkv recording file which will be used with the playback api instead of opening a device"),
    DeclareLaunchArgument(
        'recording_loop_enabled',
        default_value="false",
        description="If set to true the recording file will rewind the beginning once end of file is reached"),
    DeclareLaunchArgument(
        'recording_node',
        default_value="false",
        description="If set to true the data recorder will be enabled to publish data for our rosbags"),
    DeclareLaunchArgument(
        'skeleton_frame',
        default_value="depth_camera_link",
        description="Used by recorder for bags. Specify the frame to be used for the recorded body tracking joints"),
    DeclareLaunchArgument(
        'publish_static_coffee',
        default_value="false",
        description="If True, publish the static transforms for camera, tripod and coffee machine"),
    DeclareLaunchArgument(
        'audio_feedback',
        default_value="true",
        description="If True, publish the static transforms for camera, tripod and coffee machine"),
    DeclareLaunchArgument(
        'telegram_feedback',
        default_value="false",
        description="If True, publish the static transforms for camera, tripod and coffee machine"),
    DeclareLaunchArgument(
        'body_tracking_enabled',
        default_value="false",
        description="If set to true the joint positions will be published as marker arrays"),
    DeclareLaunchArgument(
        'body_tracking_cpu',
        default_value="false",
        description="If set to true, body tracking will work on CPU"),
    DeclareLaunchArgument(
        'body_tracking_smoothing_factor',
        default_value="0.0",
        description="Set between 0 for no smoothing and 1 for full smoothing"),
    DeclareLaunchArgument(
        'rescale_ir_to_mono8',
        default_value="false",
        description="Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied."),
    DeclareLaunchArgument(
        'ir_mono8_scaling_factor',
        default_value="1.0",
        description="Scaling factor to apply when converting IR to mono8 (see rescale_ir_to_mono8). If using illumination, use the value 0.5-1. If using passive IR, use 10."),
    DeclareLaunchArgument(
        'imu_rate_target',
        default_value="0",
        description="Desired output rate of IMU messages. Set to 0 (default) for full rate (1.6 kHz)."),
    DeclareLaunchArgument(
        'wired_sync_mode',
        default_value="0",
        description="Wired sync mode. 0: OFF, 1: MASTER, 2: SUBORDINATE."),
    DeclareLaunchArgument(
        'subordinate_delay_off_master_usec',
        default_value="0",
        description="Delay subordinate camera off master camera by specified amount in usec."),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        parameters=[
            {'depth_enabled': launch.substitutions.LaunchConfiguration('depth_enabled')},
            {'depth_mode': launch.substitutions.LaunchConfiguration('depth_mode')},
            {'depth_unit': launch.substitutions.LaunchConfiguration('depth_unit')},
            {'color_enabled': launch.substitutions.LaunchConfiguration('color_enabled')},
            {'color_format': launch.substitutions.LaunchConfiguration('color_format')},
            {'color_resolution': launch.substitutions.LaunchConfiguration('color_resolution')},
            {'fps': launch.substitutions.LaunchConfiguration('fps')},
            {'point_cloud': launch.substitutions.LaunchConfiguration('point_cloud')},
            {'rgb_point_cloud': launch.substitutions.LaunchConfiguration('rgb_point_cloud')},
            {'point_cloud_in_depth_frame': launch.substitutions.LaunchConfiguration('point_cloud_in_depth_frame')},
            {'sensor_sn': launch.substitutions.LaunchConfiguration('sensor_sn')},
            {'recording_file': launch.substitutions.LaunchConfiguration('recording_file')},
            {'recording_loop_enabled': launch.substitutions.LaunchConfiguration('recording_loop_enabled')},
            {'body_tracking_enabled': launch.substitutions.LaunchConfiguration('body_tracking_enabled')},
            {'body_tracking_cpu': launch.substitutions.LaunchConfiguration('body_tracking_cpu')},
            {'body_tracking_smoothing_factor': launch.substitutions.LaunchConfiguration('body_tracking_smoothing_factor')},
            {'rescale_ir_to_mono8': launch.substitutions.LaunchConfiguration('rescale_ir_to_mono8')},
            {'ir_mono8_scaling_factor': launch.substitutions.LaunchConfiguration('ir_mono8_scaling_factor')},
            {'imu_rate_target': launch.substitutions.LaunchConfiguration('imu_rate_target')},
            {'wired_sync_mode': launch.substitutions.LaunchConfiguration('wired_sync_mode')},
            {'subordinate_delay_off_master_usec': launch.substitutions.LaunchConfiguration('subordinate_delay_off_master_usec')}]),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='microphone_node.py',
        name='microphone_node',
        parameters = [],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("microphone_enabled"))),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='audio_labeler.py',
        name='audio_labeler_node',
        parameters = []),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='coffee_feedback.py',
        name='coffee_feedback_node',
        parameters = [
            {'audio_feedback': launch.substitutions.LaunchConfiguration('audio_feedback')},
            {'telegram_feedback': launch.substitutions.LaunchConfiguration('telegram_feedback')}],
        condition=conditions.IfCondition(use_feedback_node)
        # condition=conditions.IfCondition(
        #     launch.substitutions.PythonExpression(
        #         [launch.substitutions.LaunchConfiguration('audio_feedback'), "' or '", launch.substitutions.LaunchConfiguration('telegram_feedback')]
        #     )
        # )
    ),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='data_recorder.py',
        name='data_recorder',
        parameters=[{'skeleton_frame': launch.substitutions.LaunchConfiguration('skeleton_frame')}],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("recording_node"))),
    # If flag overwrite_robot_description is set:
    launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters = [{'robot_description' : urdf}],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    # If flag overwrite_robot_description is not set:
    launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters = [{'robot_description' : urdf}],
        remappings=remappings,
        condition=conditions.UnlessCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_path],
        remappings=remappings,
        condition=conditions.UnlessCondition(launch.substitutions.LaunchConfiguration("overwrite_robot_description"))),
    launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0','0', '1.95', '1.0471975', '0.523598', '0', 'camera_tripod', 'camera_base'],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("publish_static_coffee"))),
    launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.55','0.30', '1.22', '0', '0', '0', 'camera_tripod', 'coffee_machine'],
        condition=conditions.IfCondition(launch.substitutions.LaunchConfiguration("publish_static_coffee"))),
    ])