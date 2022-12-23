#!/usr/bin/env python3

from azure_kinect_ros_msgs.msg import AudioData
import rclpy.node
from sounddevice import InputStream, query_devices

class MicrophoneDriver(rclpy.node.Node):

    def __init__(self, device):
        super().__init__("microphone_driver")

        self.mic_pub = self.create_publisher(AudioData, 'mic_raw', 10)
        self.input_stream = InputStream(samplerate=48000, device=device, 
            dtype='int32', 
            callback=self.mic_callback)
        self.input_stream.start()

    def mic_callback(self, indata, frames, time, status):
        msg = AudioData()
        for channel in range(indata.shape[-1]):
            setattr(msg, f"channel_{channel}", indata[:, channel].tolist())
        self.mic_pub.publish(msg)

    def __del__(self) -> None:
        self.input_stream.stop()

def main(args=None):
    rclpy.init(args=args)
    index = None
    for device in query_devices():
        if "Azure Kinect Microphone Array" in device['name']:
            index = device['index']
    if not index:
        rclpy.logging.get_logger("Azure Kinect Microphone Driver").error("Azure Kinect Microphone not found")
        exit()
    else:
        node = MicrophoneDriver(index)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()