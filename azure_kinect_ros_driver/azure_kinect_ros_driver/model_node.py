#!/usr/bin/env python3

import rclpy.node
from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput
from tf import TF, pose_msg

import numpy as np
import pickle
import os
import PyKDL

from utils import markers_to_features

class ModelNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("marker_to_pose")            
        self.create_subscription(MarkerArrayStamped, "body_tracking_data", self.has_received_markers, 10)
        self.out_pub = self.create_publisher(ModelOutput, "/model_output", 10)
        
        self.features = ["26_rot",  "1_rot", "1_xy"]
        self.skeleton_frame = "camera_body"
        self.tf_utils = TF(self)
        
        file_path = os.path.dirname(os.path.realpath(__file__)) 
        with open(os.path.join(file_path, "rf_clf.pkl"), 'rb') as f:
            self.clf = pickle.load(f)


    def has_received_markers(self, msg):
        out_msg = ModelOutput()
        out_msg.header = msg.header
        if len(msg.markers)  > 0:
            msg = self.transform_skeletons(msg)
            for body_s in range(0, len(msg.markers), 32):
                body_id = msg.markers[body_s].id //100
                sample = markers_to_features(msg.markers[body_s:body_s+32], self.features)
                pred_proba = np.squeeze(self.clf.predict_proba(sample[None, :])[:, 1])
                out_msg.ids.append(body_id)
                out_msg.probas.append(pred_proba)

                out_msg.interactions.append(bool(pred_proba>0.5))
        self.out_pub.publish(out_msg)

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
    node = ModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()