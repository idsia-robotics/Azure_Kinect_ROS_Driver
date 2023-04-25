#!/usr/bin/env python3
from typing import Any

import rclpy
import rclpy.node
from azure_kinect_ros_msgs.msg import AudioData
import numpy as np
from sounddevice import OutputStream
import soundfile as sf
import std_msgs
import pickle

class Play(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("play")
        self.create_subscription(
            AudioData, 'mic_raw', self.has_received_data, 0)
        self.stream = OutputStream(48000, channels=7, 
            dtype='int32'
            )
        self.stream.start()

    def has_received_data(self, msg: AudioData) -> None:
        sample = []
        for channel in range(7):
            sample.append(getattr(msg, f"channel_{channel}"))
        sample = np.ascontiguousarray(np.array(sample).T)
        self.stream.write(sample)

    def __del__(self) -> None:
        self.stream.stop()
        



def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Play()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()