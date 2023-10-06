## Requirements
### Ubuntu
To use body tracking on GPU, you need a docker installation supporting CUDAs, like explained [here](https://linuxhint.com/use-nvidia-gpu-docker-containers-ubuntu-22-04-lts/).
Otherwise, a regular docker installation is enough for using the driver on CPU (the body tracking will be super-slow).

### Windows
To use body tracking on GPU, you need a docker installation supporting CUDAs, which we didn't test yet.
For the CPU version, we tested with [docker](https://docs.docker.com/desktop/windows/install/) using `Windows Subsystem for Linux (WSL2)` backend.
Also, you have to install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/). 

### Display in Docker
The driver won't work if a display is not available. So, a display must be set for the container, even when no GUIs are opened from it.
This means that this operations must be run for every new terminal where you want to run the container.
<details>
  <summary>UBUNTU</summary>

Simply run:
```
xhost +local:docker
```
Then launch the container as explained below.
</details>

<details>
  <summary>WINDOWS</summary>

1. Start XLaunch
2. Leave default settings and press `Next` (Multiple windows, Display number -1)
3. Leave default settings and press `Next` (Start no client)
4. Remove flag from `Native opengl` and press `Next`
5. Press `Finish`
Open a WSL terminal and launch the container as explained below:
</details>

## Dockerfile_CPU
This Dockerfile creates a container with CPU only support for the driver. The body tracking will be superslow, it's useful for debug only.

### Build
To build the container, step into the Dockerfiles folder and (I'm tagging the container just for convenience):
```
docker build -f Dockerfile_CPU . -t azure-kinect-ros:humble-cpu
```

### Run the Container
Remember to always follow the steps in [Display in Docker](#display-in-docker) first.

<details>
  <summary>UBUNTU</summary>

To check that everything works, you can run `k4aviewer` from the container.
Run:
```
docker run --privileged \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=unix$DISPLAY \
			-it azure-kinect-ros:humble-cpu \
			k4aviewer
```
The viewer should open. You will get an error because the microphone does not work. If you need, you can make it work following [this steps](https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio#pulseaudio-with-shared-socket) and then running:
```
docker run --privileged \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=unix$DISPLAY \
			--env PULSE_SERVER=unix:/tmp/pulseaudio.socket \
		    --env PULSE_COOKIE=/tmp/pulseaudio.cookie \
		    --volume /tmp/pulseaudio.socket:/tmp/pulseaudio.socket \
		    --volume /tmp/pulseaudio.client.conf:/etc/pulse/client.conf \
		    --user $(id -u):$(id -g) \
			-it azure-kinect-ros:humble-cpu \
			k4aviewer
```
If everything works (except the microphone), you can run the ROS driver. For all the available parameters, check the doc.
The following command will run the driver with body tracking enabled on CPU, so it will be super slow.
```
docker run --privileged \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=unix$DISPLAY \
			-it azure-kinect-ros:humble-cpu \
			ros2 launch azure_kinect_ros_driver driver.launch.py body_tracking_enabled:=true body_tracking_cpu:=true rectify_images:=false
```
</details>

<details>
  <summary>WINDOWS</summary>

To check that everything works, you can run `k4aviewer` from the container.
Run:
```
docker run --privileged \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=host.docker.internal:0.0 \
			-e QT_X11_NO_MITSHM=1 \
			-it azure-kinect-ros:humble-cpu \
			k4aviewer
```
If everything works (except the microphone), you can run the ROS driver. For all the available parameters, check the doc.
The following command will run the driver with body tracking enabled on CPU, so it will be super slow.
```
docker run --privileged \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=host.docker.internal:0.0 \
			-e QT_X11_NO_MITSHM=1 \
			-it azure-kinect-ros:humble-cpu \
			ros2 launch azure_kinect_ros_driver driver.launch.py body_tracking_enabled:=true body_tracking_cpu:=true rectify_images:=false
```
</details>

## Dockerfile_GPU
This Dockerfile creates a container with GPU support for the driver.

### Build
To build the container, step into the Dockerfiles folder and (I'm tagging the container just for convenience):
```
docker build -f Dockerfile_GPU . -t azure-kinect-ros:humble-gpu
```

### Run the Container
The commands are the same as for the CPU container, but we are adding `--gpus all` parameter.
Remember to always follow the steps in [Display in Docker](#display-in-docker) first.

<details>
  <summary>UBUNTU</summary>

To check that everything works, you can run `k4aviewer` from the container.
Run:
```
docker run --privileged \
			--gpus all \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=unix$DISPLAY \
			-it azure-kinect-ros:humble-gpu \
			k4aviewer
```
The viewer should open. You will get an error because the microphone does not work. If you need, you can make it work following [this steps](https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio#pulseaudio-with-shared-socket) and then running:
```
docker run --privileged \
			--gpus all \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=unix$DISPLAY \
			--env PULSE_SERVER=unix:/tmp/pulseaudio.socket \
		    --env PULSE_COOKIE=/tmp/pulseaudio.cookie \
		    --volume /tmp/pulseaudio.socket:/tmp/pulseaudio.socket \
		    --volume /tmp/pulseaudio.client.conf:/etc/pulse/client.conf \
		    --user $(id -u):$(id -g) \
			-it azure-kinect-ros:humble-gpu \
			k4aviewer
```
If everything works (except the microphone), you can run the ROS driver. For all the available parameters, check the doc.
The following command will run the driver with body tracking enabled on GPU.
```
docker run --privileged \
			--gpus all \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=unix$DISPLAY \
			-it azure-kinect-ros:humble-GPU \
			ros2 launch azure_kinect_ros_driver driver.launch.py body_tracking_enabled:=true body_tracking_cpu:=false rectify_images:=false
```
</details>

<details>
  <summary>WINDOWS</summary>

Run:
```
docker run --privileged \
			--gpus all
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=host.docker.internal:0.0 \
			-e QT_X11_NO_MITSHM=1 \
			-it azure-kinect-ros:humble-gpu \
			k4aviewer
```
If everything works (except the microphone), you can run the ROS driver. For all the available parameters, check the doc.
The following command will run the driver with body tracking enabled on GPU.
```
docker run --privileged \
			--gpus all \
			--volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
			-e DISPLAY=host.docker.internal:0.0 \
			-e QT_X11_NO_MITSHM=1 \
			-it azure-kinect-ros:humble-gpu \
			ros2 launch azure_kinect_ros_driver driver.launch.py body_tracking_enabled:=true body_tracking_cpu:=false rectify_images:=false
```
</details>
