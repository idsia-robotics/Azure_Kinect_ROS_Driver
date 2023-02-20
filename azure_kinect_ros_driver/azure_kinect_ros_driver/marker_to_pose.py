#!/usr/bin/env python3

import rclpy.node
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class MarkerToPose(rclpy.node.Node):

    def __init__(self):
        super().__init__("marker_to_pose")            
        self.create_subscription(MarkerArray, "body_tracking_data", self.has_received_markers, 10)
        self.pose_pub = self.create_publisher(PoseArray, 'body_tracking_poses', 10)
        
    def has_received_markers(self, msg):
        if len(msg.markers)  > 0:
            poses_msg = PoseArray()
            poses_msg.header = msg.markers[0].header
            poses_msg.poses = [marker.pose for marker in msg.markers if marker.id %100 in [2, 7, 26]]
            self.pose_pub.publish(poses_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()