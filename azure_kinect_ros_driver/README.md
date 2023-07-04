# Azure Kinect ROS Driver

This project is a node which publishes sensor data from the [Azure Kinect Developer Kit](https://azure.microsoft.com/en-us/services/kinect-dk/) to the [Robot Operating System (ROS)](http://www.ros.org/). Developers working with ROS can use this node to connect an Azure Kinect Developer Kit to an existing ROS installation.

This repository uses the [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) to communicate with the Azure Kinect DK. It supports both Linux and Windows installations of ROS.

[![Build Status](https://dev.azure.com/ms/Azure_Kinect_ROS_Driver/_apis/build/status/microsoft.Azure_Kinect_ROS_Driver?branchName=melodic)](https://dev.azure.com/ms/Azure_Kinect_ROS_Driver/_build/latest?definitionId=166&branchName=melodic)

## Features

This ROS node outputs a variety of sensor data, including:

- A PointCloud2, optionally colored using the color camera
- Raw color, depth and infrared Images, including CameraInfo messages containing calibration information
- Rectified depth Images in the color camera resolution
- Rectified color Images in the depth camera resolution
- The IMU sensor stream
- A TF2 model representing the extrinsic calibration of the camera

The camera is fully configurable using a variety of options which can be specified in ROS launch files or on the command line.

However, this node does ***not*** expose all the sensor data from the Azure Kinect Developer Kit hardware. It does not provide access to:

- Microphone array

For more information about how to use the node, please see the [usage guide](docs/usage.md).

## Status

This code is provided as a starting point for using the Azure Kinect Developer Kit with ROS. Community developed features are welcome.

For information on how to contribute, please see our [contributing guide](CONTRIBUTING.md).

## Building

The Azure Kinect ROS Driver uses colcon to build. For instructions on how to build the project please see the 
[building guide](docs/building.md).

## Join Our Developer Program

Complete your developer profile [here](https://aka.ms/iwantmr) to get connected with our Mixed Reality Developer Program. You will receive the latest on our developer tools, events, and early access offers.

## Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## Reporting Security Issues
Security issues and bugs should be reported privately, via email, to the
Microsoft Security Response Center (MSRC) at <[secure@microsoft.com](mailto:secure@microsoft.com)>.
You should receive a response within 24 hours. If for some reason you do not, please follow up via
email to ensure we received your original message. Further information, including the
[MSRC PGP](https://technet.microsoft.com/en-us/security/dn606155) key, can be found in the
[Security TechCenter](https://technet.microsoft.com/en-us/security/default).

## License

[MIT License](LICENSE)



## Commands for me

### Visualize body tracking as frames in rviz
> ros2 run azure_kinect_ros_driver marker_to_pose.py


### Export audio data for audio detector
Run mic node:
> ros2 run azure_kinect_ros_driver microphone_node.py

Save topic in bag file
> ros2 bag record /mic_raw

Then play the bag file while running
> ros2 run azure_kinect_ros_driver play_audio.py

then, publish True to record topic to store the file, after the bag has finished
> ros2 topic pub /record std_msgs/msg/Empty "{}" --once

### Calibrate audio detector
Listen to audio file
mark t0 when the sound is started for sure, and t1 when the timer is for sure going on, but almost finished

```
s0 = t0*48000 = starting sample
s1 = t1 *48000 = ending sample
import numpy as np
data = np.load("recorded.pkl", allow_pickle=True)
refdata = data[s0:s1, :]
np.save("ref.npy", refdata)
```


### Collect Data
```
ros2 bag record /body_tracking_data_rec /audio_label_rec -b 100000000
```
while running the sensor with:
```
ros2 launch azure_kinect_ros_driver driver.launch.py body_tracking_enabled:=true fps:=5 depth_mode:="WFOV_2X2BINNED" microphone_enabled:=true recording_node:=true audio_feedback:=True telegram_feedback:=False skeleton_frame:='camera_body'
```
OR ?
> ros2 launch azure_kinect_ros_driver driver.launch.py body_tracking_enabled:=true fps:=5 depth_mode:="WFOV_UNBINNED" microphone_enabled:=true recording_node:=true audio_feedback:=True telegram_feedback:=False