# Switch to ORBBEC Femto Cameras 

In order to use any ORBBEC Femto camera in place of the Microsoft Azure Kinect, ORBBEC designed a wrapper allows exactly that.
The tested prebuilt version can be found here: [OrbbecSDK-K4A-Wrapper v1.10.1](https://github.com/orbbec/OrbbecSDK-K4A-Wrapper/releases/tag/v1.10.1)


Once downloaded and unzipped in the folder **doc** there will be a PDF with all the instructions needed to accomplish the switch. 
If the Azure Kinect SDK is already installed on the user PC, the switch consist in only moving the libraries files in the **lib** folder (provided by ORBBEC) into **/usr/lib/x86_64-linux-gnu** (substituting the ones provided by Microsoft).
In the tested version there is a typo in the file naming, please change **libk4arecord.so.1.4.0** into **libk4arecord.so.1.4.1**.


If the user wants to have the possibility to use both Azure Kinect and ORBBEC Femto cameras the script **k4a_switch_kinect_femto.sh** can be used.
In this case the Microsoft libraries needs to be manually placed in the folder **/usr/lib/x86_64-linux-gnu/k4a_microsoft/** with the ORBBEC libraries in the folder **/usr/lib/x86_64-linux-gnu/k4a_orbbec/**.
The script invoked with different arguments will put the correct libraries when needed.
The two cameras cannot be used simultaneously.

ORBBEC Femto Cameras use a different timestamp with respect to Azure Kinect, this discrepancy will cause the ROS timestamps to be off. This issue has been solved in the provided **azure_kinect_ros_driver driver.launch.py**. The default is still the Azure Kinect however the Femto camera timestamp fix can be used by specifying the launch argument **camera_type:="femto"**.