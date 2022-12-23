#!/usr/bin/env python3

import rclpy.node
from azure_kinect_ros_msgs.msg import AudioData
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from utils import Detector
from collections import deque
import numpy as np
from tf import TF, pose_msg
import PyKDL

class DataRecorder(rclpy.node.Node):

    def __init__(self):
        super().__init__("data_recorder")
        topic_to_rec = {"audio_label": Bool,
                        # "mic_raw": AudioData,
                        # "tf_static": TFMessage, 
                        # "tf": TFMessage, 
                        "body_tracking_data": MarkerArray}
        self.repubs = {}
        self.skeleton_frame = self.declare_parameter("skeleton_frame", "camera_tripod").value
        self.tf_utils = TF(self)
        for topic, msg_type in topic_to_rec.items():
            self.create_subscription(msg_type, topic, self.create_topic_cb(topic), 10)
            self.repubs[topic] = self.create_publisher(msg_type, topic + "_rec", 10)
        self.record = False
        
    def create_topic_cb(self, topic_name):
        def topic_cb(msg):
            if topic_name == "body_tracking_data":
                if len(msg.markers) == 0:
                    self.record = False
                else:
                    self.record = True
                    # TODO: update msg frame
                    if self.skeleton_frame != 'depth_camera_link':
                        msg = self.transform_skeletons(msg)
            if self.record:
                self.repubs[topic_name].publish(msg)
        return topic_cb

    def transform_skeletons(self, msg):
        tform = self.tf_utils.get_transform(msg.markers[0].header.frame_id, self.skeleton_frame)
        for i in range(len(msg.markers)):
            pose = msg.markers[i].pose
            pos = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
            rot = PyKDL.Rotation.Quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            msg.markers[i].pose = pose_msg(tform * PyKDL.Frame(rot, pos)).pose
            msg.markers[i].header.frame_id = self.skeleton_frame
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()