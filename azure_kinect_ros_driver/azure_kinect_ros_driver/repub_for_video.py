#!/usr/bin/env python3

import rclpy.node
from visualization_msgs.msg import MarkerArray
import numpy as np

class RepubForVideo(rclpy.node.Node):

    def __init__(self):
        super().__init__("video_repub")
        self.create_subscription(MarkerArray, "body_tracking_data_rec", self.cb, 10)
        self.repub = self.create_publisher(MarkerArray, "body_tracking_data_video", 10)
    
    def cb(self, msg):
        if len(msg.markers) != 0:
            for i in range(len(msg.markers)):
                r, g, b = 0.5, 0.5, 0.5
                if msg.markers[i].id//100 in [3, 35]:
                    r, g, b = 0., 1., 0.
                msg.markers[i].color.r = r
                msg.markers[i].color.g = g
                msg.markers[i].color.b = b
        self.repub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RepubForVideo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()