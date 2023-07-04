## Azure Kinect SDK on Ubuntu 22.04
https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1790
### Install libsoundio1
__libsoundio1__ is not available for Ubuntu 22, but we can easily install it from  Ubuntu 20.04 focal apt repos.
First, we need to modify `/etc/apt/sources.list`.
```
# backup current sources.list to revert later
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bu  
sudo gedit /etc/apt/sources.list
```
Then replace the file content with this (copy-paste):
```
# deb cdrom:[Ubuntu 22.04.1 LTS _Jammy Jellyfish_ - Release amd64 (20220809.1)]/ jammy main restricted

# See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
# newer versions of the distribution.
deb http://ch.archive.ubuntu.com/ubuntu/ focal main restricted
# deb-src http://ch.archive.ubuntu.com/ubuntu/ jammy main restricted

## Major bug fix updates produced after the final release of the
## distribution.
deb http://ch.archive.ubuntu.com/ubuntu/ focal-updates main restricted
# deb-src http://ch.archive.ubuntu.com/ubuntu/ jammy-updates main restricted

## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu
## team. Also, please note that software in universe WILL NOT receive any
## review or updates from the Ubuntu security team.
deb http://ch.archive.ubuntu.com/ubuntu/ focal universe
# deb-src http://ch.archive.ubuntu.com/ubuntu/ jammy universe
deb http://ch.archive.ubuntu.com/ubuntu/ jammy-updates universe
# deb http://ch.archive.ubuntu.com/ubuntu/ focal-updates universe
# deb-src http://ch.archive.ubuntu.com/ubuntu/ jammy-updates universe

## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu 
## team, and may not be under a free licence. Please satisfy yourself as to 
## your rights to use the software. Also, please note that software in 
## multiverse WILL NOT receive any review or updates from the Ubuntu
## security team.
deb http://ch.archive.ubuntu.com/ubuntu/ jammy multiverse
# deb-src http://ch.archive.ubuntu.com/ubuntu/ jammy multiverse
deb http://ch.archive.ubuntu.com/ubuntu/ jammy-updates multiverse
# deb-src http://ch.archive.ubuntu.com/ubuntu/ jammy-updates multiverse

## N.B. software from this repository may not have been tested as
## extensively as that contained in the main release, although it includes
## newer versions of some applications which may provide useful features.
## Also, please note that software in backports WILL NOT receive any review
## or updates from the Ubuntu security team.
deb http://ch.archive.ubuntu.com/ubuntu/ jammy-backports main restricted universe multiverse
# deb-src http://ch.archive.ubuntu.com/ubuntu/ jammy-backports main restricted universe multiverse

deb http://security.ubuntu.com/ubuntu jammy-security main restricted
# deb-src http://security.ubuntu.com/ubuntu jammy-security main restricted
deb http://security.ubuntu.com/ubuntu jammy-security universe
# deb-src http://security.ubuntu.com/ubuntu jammy-security universe
deb http://security.ubuntu.com/ubuntu jammy-security multiverse
# deb-src http://security.ubuntu.com/ubuntu jammy-security multiverse

# This system was installed using small removable media
# (e.g. netinst, live or single CD). The matching "deb cdrom"
# entries were disabled at the end of the installation process.
# For information about how to configure apt package sources,
# see the sources.list(5) manual.
```
Save and close the file. Note that we have simply replaced the first 3 jammy sources with focal ones.
Now, we should be able to install __libsoundio1__:
```
sudo apt-get update
sudo apt-get install libsoundio1
```
Then, we can revert the source file: 
```
sudo rm /etc/apt/sources.list
sudo mv /etc/apt/sources.list.bu /etc/apt/sources.list
sudo apt-get update
```
### Install SDK
Since the SDK is not officially available for  Ubuntu 22.04, we need to download the `.deb` packages and install them directly.
Download these:
	- [libk4a1.4_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb)
	- [libk4a1.4_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb)
	- [k4a-tools_1.4.1_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb)
Then step into the download folder and run:
```
sudo dpkg -i libk4a1.4_1.4.1_amd64.deb 
sudo dpkg -i libk4a1.4-dev_1.4.1_amd64.deb 
sudo dpkg -i k4a-tools_1.4.1_amd64.deb
```
Finally, set the permission for the device:
- copy [99-k4a.rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules) file into `/etc/udev/rules.d`

Now power up the device and connect it via USB. If everything is working, you should be able to open a terminal and run:
```
k4aviewer
```
There you can open the device and check that all the inputs are running.
### Install Body Tracking SDK
I don't have a GPU to test the installation with CUDA, according [this](https://learn.microsoft.com/en-us/azure/kinect-dk/body-sdk-setup), you have to make sure you have the latest NVIDIA driver installed. Then, again, we have to download the `.deb` files and install them directly. These are:
- [libk4abt1.1_1.1.2_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb)
- [libk4abt1.1-dev_1.1.2_amd64.deb](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb)

Install them:
```
sudo dpkg -i libk4abt1.1_1.1.2_amd64.deb 
sudo dpkg -i libk4abt1.1-dev_1.1.2_amd64.deb 
```
Now, you can test the installation with:
```
k4abt_simple_3d_viewer
```
In my case, I had to run it like this to use the CPU:
```
k4abt_simple_3d_viewer CPU
```

