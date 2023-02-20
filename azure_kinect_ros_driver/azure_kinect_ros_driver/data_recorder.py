#!/usr/bin/env python3

import rclpy.node
from azure_kinect_ros_msgs.msg import AudioData
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from utils import Detector
from collections import deque
import numpy as np
from tf import TF, pose_msg
import PyKDL
import cv2
from utils import image_to_numpy
from datetime import datetime

class DataRecorder(rclpy.node.Node):

    def __init__(self):
        super().__init__("data_recorder")
        topic_to_rec = {"audio_label": Bool,
                        "audio_label_dist": Float32,
                        # "mic_raw": AudioData,
                        # "tf_static": TFMessage, 
                        # "tf": TFMessage, 
                        "body_tracking_data": MarkerArray}
        self.repubs = {}
        self.skeleton_frame = self.declare_parameter("skeleton_frame", "depth_camera_link").value
        self.tf_utils = TF(self)
        for topic, msg_type in topic_to_rec.items():
            self.create_subscription(msg_type, topic, self.create_topic_cb(topic), 10)
            self.repubs[topic] = self.create_publisher(msg_type, topic + "_rec", 10)
        self.create_subscription(Image, "/rgb/image_raw", self.has_received_image, 1)
        self.record = False
        self.write_png = False
        self.time = 0.0
        self.cooldown = 1.
        
    def create_topic_cb(self, topic_name):
        def topic_cb(msg):
            if topic_name == "body_tracking_data":
                if len(msg.markers) == 0:
                    self.time += 0.25
                    if self.time > self.cooldown:
                        self.record = False
                        self.time = 0.0                        
                else:
                    if not self.record:
                        self.record = True
                        self.write_png = True
                        self.time = 0.0
                    # TODO: update msg frame
                    if self.skeleton_frame != 'depth_camera_link':
                        msg = self.transform_skeletons(msg)
                if self.record and len(msg.markers) > 0:
                    self.repubs[topic_name].publish(msg)
            else:
                if self.record:
                    self.repubs[topic_name].publish(msg)
        return topic_cb

    def has_received_image(self, msg):
        if self.write_png:
            img = image_to_numpy(msg)
            cv2.imwrite(f'/home/idsia/png/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.png', cv2.resize(img, (int(img.shape[1]*0.25), int(img.shape[0]*0.25))))
            self.write_png = False

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