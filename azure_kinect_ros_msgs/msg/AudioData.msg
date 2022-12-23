# Raw audio samples
# .. note::
#   Unlike ROS1, ROS2 has no ``audio_common_msgs``, so for now we define our own format
#   to publish the raw 16-bit 48Khz audio stream from the kinect  7 channel microphone

std_msgs/Header header  # the timestamp and frame of the microphone recording
int32[] channel_0            # an array of 16-bit audio samples
int32[] channel_1
int32[] channel_2
int32[] channel_3
int32[] channel_4
int32[] channel_5
int32[] channel_6