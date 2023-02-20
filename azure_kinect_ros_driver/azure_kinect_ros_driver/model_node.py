#!/usr/bin/env python3

import rclpy.node
from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput

import numpy as np
import pickle

from utils import markers_to_features

class ModelNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("marker_to_pose")            
        self.create_subscription(MarkerArraySyamped, "body_tracking_data", self.has_received_markers, 10)
        self.out_pub = self.create_publisher(ModelOutput, "/model_output", 10)
        
        self.features = ["26_rot",  "1_rot", "1_xy"]
        
        with open("rf_clf.pkl", 'rb') as f:
            self.clf = pickle.load(f)


    def has_received_markers(self, msg):
        if len(msg.markers)  > 0:
            out_msg = ModelOutput()
            out_msg.header = msg.header
            for body_s in range(0, len(msg.markers), 32):
                body_id = msg.markers[body_s].id %100
                sample = markers_to_features(msg.markers[body_s:body_s+32], self.features)
                pred_proba = clf.predict_proba(sample)[:, 1]
                out_msg.ids.append(body_id)
                out_msg.probas.append(pred_proba)
                out_msg.interactions.append(pred_proba>0.5)




def main(args=None):
    rclpy.init(args=args)
    node = ModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()