#!/usr/bin/env python3

from azure_kinect_ros_msgs.msg import AudioData
import rclpy.node
from std_msgs.msg import Bool
from utils import Detector
from collections import deque
import numpy as np

class AudioLabeler(rclpy.node.Node):

    def __init__(self):
        super().__init__("audio_label")
        self.chunksize = self.declare_parameter('audio_detector_chunksize', 4800).value
        self.samples = deque(maxlen=self.chunksize)
        self.detector = Detector(chunksize=self.chunksize)
        self.create_subscription(AudioData, 'mic_raw', self.has_received_data, 10)
        self.label_pub = self.create_publisher(Bool, 'audio_label', 10)
        self.create_timer(0.1, self.compute_label)

    def has_received_data(self, msg):
        packet = []
        for channel in range(7):
            packet.append(getattr(msg, f"channel_{channel}"))
        self.samples.extend(np.array(packet).T)

    def compute_label(self):
        label = False
        if len(self.samples) == self.chunksize:
            chunk = np.vstack(self.samples.copy())
            self.samples.clear()
            label = self.detector.consume_chunk(chunk)
        self.label_pub.publish(Bool(data=label))

def main(args=None):
    rclpy.init(args=args)
    node = AudioLabeler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()