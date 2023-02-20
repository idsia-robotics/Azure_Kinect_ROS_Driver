#!/usr/bin/env python3

import rclpy.node
from visualization_msgs.msg import MarkerArray
from azure_kinect_ros_msgs.msg import MarkerArrayStamped
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from utils import image_to_numpy, numpy_to_image, Calibration, get_body_segments
import cv2

class SkeletonToRGB(rclpy.node.Node):

    def __init__(self):
        super().__init__("skeleton_to_rgb")


        # self.create_subscription(MarkerArray, "/body_tracking_data", self.markers_cb, 1)
        # self.markers_pub = self.create_publisher(MarkerArrayStamped, "/body_tracking_data_stamped", 1)

        qos = rclpy.qos.QoSProfile(
            depth=1,
            # durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            # reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.rgb_sub = Subscriber(self, Image, "/rgb/image_raw", qos_profile=qos)
        self.skeletons_sub = Subscriber(self, MarkerArrayStamped, "/body_tracking_data", qos_profile=qos)
        self.synchronizer = ApproximateTimeSynchronizer([self.rgb_sub, self.skeletons_sub],
                                                        queue_size=1, slop=0.1)

        self.image_pub = self.create_publisher(Image, "/rgb/image_skeleton", 1)

        self.synchronizer.registerCallback(self.topic_sync_cb)
        self.calibration = Calibration(2)
        self.get_logger().info("ready")

    # def markers_cb(self, m_msg):
    #     if len(m_msg.markers)>0:
    #         msg = MarkerArrayStamped()
    #         msg.header = m_msg.markers[0].header
    #         msg.markers = m_msg.markers
    #         self.markers_pub.publish(msg)
        
    def topic_sync_cb(self, rgb_msg, skeleton_msg):
        if len(skeleton_msg.markers) != 0:
            color = (255, 0, 0)
            image = image_to_numpy(rgb_msg)
            points_3d = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in skeleton_msg.markers])
            points_3d *= 1000.
            points_2d = np.squeeze(self.calibration.depth_to_rgb_image(points_3d))
            for body_segment in get_body_segments():
                for i in range(len(body_segment)-1):
                    image = cv2.line(image, points_2d[body_segment[i]].astype(int), points_2d[body_segment[i+1]].astype(int), color, 2)
            for point in points_2d:
                image = cv2.circle(image, point.astype(int), 3, color, 3)
            image = numpy_to_image(image, rgb_msg.encoding)
            self.image_pub.publish(image)


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonToRGB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()