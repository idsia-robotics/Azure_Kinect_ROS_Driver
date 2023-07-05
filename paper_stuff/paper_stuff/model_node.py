import rclpy.node
from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput
from .tf import TF, pose_msg

import numpy as np
import pickle
import os
import PyKDL
from .utils import markers_to_features
import torch
from .pt_model import LSTM
from ament_index_python.packages import get_package_share_directory

class ModelNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("model_node")
        self.out_pub = self.create_publisher(ModelOutput, "/model_output", 10)
        self.features = ["26_rot",  "1_rot", "1_xy"]
        self.skeleton_frame = self.declare_parameter("skeleton_frame", "camera_base").value
        self.pred_th = self.declare_parameter("prediction_threshold", 0.3).value
        self.create_subscription(MarkerArrayStamped, "body_tracking_data", self.has_received_markers, 10)
        self.tf_utils = TF(self)
        self.tform = None
        self.pkg_path = get_package_share_directory("paper_stuff")
        
    def transform_skeletons(self, msg: MarkerArrayStamped) -> MarkerArrayStamped:
        for i in range(len(msg.markers)):
            pose = msg.markers[i].pose
            pos = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
            rot = PyKDL.Rotation.Quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            msg.markers[i].pose = pose_msg(self.tform * PyKDL.Frame(rot, pos)).pose
            msg.markers[i].header.frame_id = self.skeleton_frame
        return msg

    def has_received_markers(self, msg: MarkerArrayStamped) -> None:
        out_msg = ModelOutput()
        out_msg.header = msg.header
        
        if len(msg.markers)>0:
            if self.tform is None:
                self.tform = self.tf_utils.get_transform(msg.markers[0].header.frame_id, self.skeleton_frame)
            if self.tform is not None:
                msg = self.transform_skeletons(msg)
                out_msg = self.predict(msg, out_msg)

        self.out_pub.publish(out_msg)

    def predict(self, msg: MarkerArrayStamped, out_msg: ModelOutput) -> ModelOutput:
        pass

class RandomForestNode(ModelNode):
    def __init__(self):
        super().__init__()
        self.features = ["26_rot",  "1_rot", "1_xy"]
        with open(self.pkg_path + "/models/rf_clf.pkl", 'rb') as f:
            self.model = pickle.load(f)

    def predict(self, msg: MarkerArrayStamped, out_msg: ModelOutput) -> ModelOutput:
        for body_s in range(0, len(msg.markers), 32):
            body_id = msg.markers[body_s].id //100
            sample = markers_to_features(msg.markers[body_s:body_s+32], self.features)
            pred_proba = np.squeeze(self.model.predict_proba(sample[None, :])[:, 1])
            out_msg.ids.append(body_id)
            out_msg.probas.append(pred_proba)
            out_msg.interactions.append(bool(pred_proba>self.pred_th))
        return out_msg

class LSTMNode(ModelNode):
    def __init__(self):
        super().__init__()
        self.features = ["26_rot",  "1_rot", "1_xy"]#, '1_dist']
        file_path = self.pkg_path + "/models/lstm.info"
        with open(file_path) as f:
            info = eval(f.read())
        # with open(os.path.join(file_path, "lstm.pkl"), 'rb') as f:
        #     info = pickle.load(f)
        self.device = torch.device('cpu')
        self.model = LSTM(input_size=info['input_size'], hidden_size=info['hidden_size'], out_size=1,
                        num_layers=info['num_layers']).to(self.device)
        
        self.model.load_state_dict(torch.load(self.pkg_path + "/models/lstm.pth", map_location=torch.device('cpu')))
        self.model.eval()
        self.hiddens = {}
        self.create_timer(1., self.check_hiddens)

    def check_hiddens(self):
        for body_id in list(self.hiddens.keys()):
            self.hiddens[body_id][1] += 1.
            if  self.hiddens[body_id][1] >= 5.:
                del self.hiddens[body_id]

    def predict(self, msg: MarkerArrayStamped, out_msg: ModelOutput) -> ModelOutput:
        with torch.no_grad():
            for body_s in range(0, len(msg.markers), 32):
                body_id = msg.markers[body_s].id //100
                if body_id not in self.hiddens:
                    self.hiddens[body_id] = [self.model.init_hidden(), 0]
                sample = torch.from_numpy(markers_to_features(msg.markers[body_s:body_s+32], self.features)[None, :]).type(torch.float32)
                out, self.hiddens[body_id][0] = self.model(sample, self.hiddens[body_id][0])
                self.hiddens[body_id][1] = 0.
                pred_proba = torch.sigmoid(out)
                out_msg.ids.append(body_id)
                out_msg.probas.append(pred_proba)
                out_msg.interactions.append(bool(pred_proba>self.pred_th))
            return out_msg