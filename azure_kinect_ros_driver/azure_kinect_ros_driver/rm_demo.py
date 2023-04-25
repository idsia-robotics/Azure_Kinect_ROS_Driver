#!/usr/bin/env python3

import rclpy.node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import TF

from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput
from utils import angle_difference, PID
from robomaster_msgs.msg import LEDEffect

import numpy as np
from scipy.spatial.transform import Rotation as R
import PyKDL
import matplotlib.cm as cm


class RoboMasterDemo(rclpy.node.Node):

    def __init__(self):
        super().__init__("rm_demo")

        qos = rclpy.qos.QoSProfile(
            depth=1,
            # durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            # reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )
        # self.rgb_sub = Subscriber(self, Image, "/rgb/image_raw", qos_profile=qos)
        self.skeletons_sub = Subscriber(self, MarkerArrayStamped, "/body_tracking_data", qos_profile=qos)
        self.model_sub = Subscriber(self, ModelOutput,"model_output", qos_profile=qos)
        self.synchronizer = ApproximateTimeSynchronizer([self.skeletons_sub, self.model_sub],
                                                        queue_size=1, slop=0.1)

        # self.image_pub = self.create_publisher(Image, "/rgb/image_skeleton", 1)
        # self.plot_bb = True
        self.synchronizer.registerCallback(self.topic_sync_cb)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.current_yaw = None

        # self.create_subscription(MarkerArrayStamped, "body_tracking_data", self.has_received_markers, 1)
        self.target_yaw = 0.0

        self.dt = 0.1
        self.create_timer(0.1, self.rotate_rm)

        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tf_utils = TF(self)
        self.tform = None
        self.skeleton_frame = "camera_base"
        self.PID = PID(2, 0.25, 0.1)
        self.led_msg = LEDEffect()
        self.led_pub = self.create_publisher(LEDEffect, "/leds/effect", 10)

        self.get_logger().info("ready")

    def odom_cb(self, msg):
        ori = msg.pose.pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        yaw, _, _ = R.from_quat(quat).as_euler('zxy', degrees=False)
        if self.current_yaw is None:
            self.zero = yaw
        self.current_yaw = yaw - self.zero

    def rotate_rm(self):
        cmd = Twist()
        if self.current_yaw is not None:
            cmd.angular.z = self.PID.step(angle_difference(self.target_yaw, self.current_yaw), self.dt)
        self.vel_pub.publish(cmd)
        self.led_pub.publish(self.led_msg)

    def topic_sync_cb(self, markers_msg, model_msg):
        if self.tform is None and len(markers_msg.markers)>0:
                self.tform = self.tf_utils.get_transform(markers_msg.markers[0].header.frame_id, self.skeleton_frame)

        target_yaw = 0.0
        led_msg = LEDEffect()
        if len(model_msg.ids)>0:
            max_id = np.argmax(model_msg.probas)
            if model_msg.probas[max_id]>0.43:
                if self.tform is not None:
                    markers = markers_msg.markers
                    pose = None
                    for body_s in range(0, len(markers), 32):
                        if markers[body_s].id //100 == model_msg.ids[max_id]:
                            pose = markers[body_s+1].pose
                            break
                    if pose is not None:
                        pos = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
                        rot = PyKDL.Rotation.Quaternion(
                            pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w)
                        pos = (self.tform * PyKDL.Frame(rot, pos)).p
                        pos = np.array([pos.x(), pos.y()])
                        v = pos - np.array([0.40, 0.0])
                        target_yaw = np.arctan2(v[1], v[0])
                        r, g, b, _ = cm.viridis(model_msg.probas[max_id])
                        led_msg.color = ColorRGBA(r=r, g=g, b=b, a=model_msg.probas[max_id])
                        led_msg.effect = LEDEffect.ON
        self.target_yaw = target_yaw
        self.led_msg = led_msg
        self.get_logger().info(f"target yaw: {target_yaw}")
        
    # def topic_sync_cb(self, rgb_msg, skeleton_msg, model_msg):
    #     image = image_to_numpy(rgb_msg)
    #     if len(skeleton_msg.markers) != 0:
    #         points_3d = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in skeleton_msg.markers])
    #         points_3d *= 1000.
    #         points_2d = np.squeeze(self.calibration.depth_to_rgb_image(points_3d))
    #         points_2d = points_2d.astype(int).clip([0, 0], image.shape[1::-1])
    #         for body_s in range(0, len(points_2d), 32):
    #             body_id = skeleton_msg.markers[body_s].id //100
    #             proba = 0.
    #             for j, bid in enumerate(model_msg.ids):
    #                 if bid == body_id:
    #                     proba = model_msg.probas[j]
    #                     break
    #             # color = [0, 255, 0]#[(body_id*10)%255]*3
    #             color = (np.array(cm.viridis(proba))[:-1]*255) #[0, 255*label, 255*(not label)]#[(body_id*10)%255]*3
    #             color = (int(color[2]), int(color[1]), int(color[0]))
    #             if self.plot_bb:
    #                 rect = cv2.boundingRect(points_2d[body_s:body_s+32])
    #                 image = cv2.rectangle(image, rect, color, 3)
    #                 image = cv2.putText(image, f"{proba:.2f}", (rect[0]+rect[2]-80, rect[1]+40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)
    #             else:
    #                 for body_segment in get_body_segments():
    #                     for i in range(len(body_segment)-1):
    #                         image = cv2.line(image, 
    #                                         points_2d[body_s + body_segment[i]], 
    #                                         points_2d[body_s + body_segment[i+1]], 
    #                                         color, 2)
    #                 for point in points_2d[body_s:body_s+32]:
    #                     image = cv2.circle(image, point, 3, color, 3)
                        

    #     cv2.imshow('skeleton_rgb', image)
    #     cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RoboMasterDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()