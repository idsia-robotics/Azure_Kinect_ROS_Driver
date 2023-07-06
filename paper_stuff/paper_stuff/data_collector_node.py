import rclpy.node
from rclpy.duration import Duration
from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput, Label
from .tf import TF, pose_msg
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import numpy as np
import pickle
import os
import PyKDL
from .utils import markers_to_features, transform_skeletons
from scipy.spatial.transform import Rotation as R
import chime
chime.theme('big-sur')

class DataCollectorNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("data_collector_node")
        # self.features_pub = self.create_publisher(PoseArray, "/features_poses", 10)
        # self.poses_pub = self.create_publisher(PoseArray, "/raw_poses", 10)
        self.skeleton_frame = self.declare_parameter("skeleton_frame", "camera_base").value
        self.create_subscription(MarkerArrayStamped, "body_tracking_data", self.has_received_markers, 10)
        self.create_subscription(Empty, "key_pressed", self.key_pressed, 10)
        self.body_pub = self.create_publisher(MarkerArrayStamped, "body_tracking_transformed", 10)
        self.label_pub = self.create_publisher(Label, "label", 10)
        self.state_pub = self.create_publisher(Image, "color_state", 1)
        self.img_msg = Image()
        self.img_msg.encoding = 'rgb8'
        self.img_msg.width = 2
        self.img_msg.height = 2
        self.img_msg.step = self.img_msg.width*3
        self.r = np.zeros((self.img_msg.height, self.img_msg.width, 3), dtype=np.uint8)
        self.r[:,:, 0] = 255
        self.g = np.zeros((self.img_msg.height, self.img_msg.width, 3), dtype=np.uint8)
        self.g[:,:, 1] = 255
        self.tf_utils = TF(self)
        self.tform = None
        self.state = "start"
        self.run_timer = self.create_timer(0.1, self.run)
        self.idx = 0
        self.body_id = None
        self.track_timer = None

    def run(self):
        if self.state == "start":
            self.get_logger().info("start interaction")
            self.state = "waiting_body"
            chime.error()
            self.img_msg.data = self.g.flatten().tolist()
            self.state_pub.publish(self.img_msg)
        elif self.state == "tracking":
            self.img_msg.data = (self.r+self.g).flatten().tolist()
            self.state_pub.publish(self.img_msg)
            if self.get_clock().now() - self.track_timer >= Duration(seconds=5):
                lbl_msg = Label()
                lbl_msg.sequence_id = self.idx
                lbl_msg.body_id = self.body_id
                lbl_msg.label = False
                lbl_msg.header.stamp = self.get_clock().now().to_msg()
                self.label_pub.publish(lbl_msg)
                self.idx += 1
                self.body_id = None
                self.track_timer = self.get_clock().now()
                self.get_logger().info("Tracking lost, ending sequence")
                self.state = "start"
        elif self.state == "cooldown":
            if self.get_clock().now() - self.track_timer >= Duration(seconds=5):
                self.state = "start"
                self.track_timer = None
        elif self.state == "interacted":
            self.state = "cooldown"
            lbl_msg = Label()
            lbl_msg.sequence_id = self.idx
            lbl_msg.body_id = self.body_id
            lbl_msg.label = True
            lbl_msg.header.stamp = self.get_clock().now().to_msg()
            self.label_pub.publish(lbl_msg)
            self.idx += 1
            self.body_id = None
            self.track_timer = self.get_clock().now()
            self.get_logger().info("Interaction, ending sequence")
            chime.success()
            self.img_msg.data = self.r.flatten().tolist()
            self.state_pub.publish(self.img_msg)

    def key_pressed(self, msg: Empty) -> None:
        if self.state == "tracking":
            self.state = "interacted"

    def has_received_markers(self, msg: MarkerArrayStamped) -> None:
        # self.get_logger().info(self.state)
        if len(msg.markers)>0:
            if self.tform is None:
                self.tform = self.tf_utils.get_transform(msg.markers[0].header.frame_id, self.skeleton_frame)
            if self.tform is not None:
                msg = transform_skeletons(msg, self.skeleton_frame, self.tform)
                if self.state == "waiting_body":
                    self.state = "tracking"
                    self.track_timer = self.get_clock().now()
                if self.state == "tracking":
                    self.body_id = msg.markers[0].id //100
                    self.track_timer = self.get_clock().now()
                    self.body_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()