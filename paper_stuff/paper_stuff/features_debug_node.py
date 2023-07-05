import rclpy.node
from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput
from .tf import TF, pose_msg
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import pickle
import os
import PyKDL
from .utils import markers_to_features
from scipy.spatial.transform import Rotation as R

class FeaturesDebugNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("features_debug_node")
        self.features_pub = self.create_publisher(PoseArray, "/features_poses", 10)
        self.poses_pub = self.create_publisher(PoseArray, "/raw_poses", 10)
        self.features = ["1_rot", "1_xy", "26_rot", "26_xy"]
        self.skeleton_frame = self.declare_parameter("skeleton_frame", "camera_base").value
        self.create_subscription(MarkerArrayStamped, "body_tracking_data", self.has_received_markers, 10)
        self.tf_utils = TF(self)
        self.tform = None
        
    def transform_skeletons(self, msg: MarkerArrayStamped) -> MarkerArrayStamped:
        msg.header.frame_id = self.skeleton_frame
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
        if len(msg.markers)>0:
            if self.tform is None:
                self.tform = self.tf_utils.get_transform(msg.markers[0].header.frame_id, self.skeleton_frame)
            if self.tform is not None:
                msg = self.transform_skeletons(msg)
                for body_s in range(0, len(msg.markers), 32):
                    body_id = msg.markers[body_s].id //100

                    # torso_sin/cos, torso_xy, head_sin/cos, head_xy
                    sample = markers_to_features(msg.markers[body_s:body_s+32], self.features)
                    pose_arr = PoseArray()
                    pose_arr.header = msg.header
                    torso_pose = Pose()
                    quat = R.from_euler('z', np.arctan2(sample[0], sample[1]), degrees=False).as_quat()
                    torso_pose.orientation.x = quat[0]
                    torso_pose.orientation.y = quat[1]
                    torso_pose.orientation.z = quat[2]
                    torso_pose.orientation.w = quat[3]
                    torso_pose.position.x = sample[2]
                    torso_pose.position.y = sample[3]
                    pose_arr.poses.append(torso_pose)

                    head_pose = Pose()
                    quat = R.from_euler('z', np.arctan2(sample[4], sample[5]), degrees=False).as_quat()
                    head_pose.orientation.x = quat[0]
                    head_pose.orientation.y = quat[1]
                    head_pose.orientation.z = quat[2]
                    head_pose.orientation.w = quat[3]
                    head_pose.position.x = sample[6]
                    head_pose.position.y = sample[7]
                    pose_arr.poses.append(head_pose)
                    self.features_pub.publish(pose_arr)

                    pose_arr = PoseArray()
                    for marker in msg.markers[body_s:body_s+1]:
                        pose_arr.header = msg.header
                        pose = marker.pose
                        pose_arr.poses.append(pose)
                    self.poses_pub.publish(pose_arr)




def main(args=None):
    rclpy.init(args=args)
    node = FeaturesDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()