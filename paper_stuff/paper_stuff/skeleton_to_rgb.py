import rclpy.node
from rclpy.duration import Duration
from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from .utils import Calibration, get_body_segments
import cv2
import cv_bridge
import matplotlib.cm as cm

class SkeletonToRGB(rclpy.node.Node):

    def __init__(self):
        super().__init__("skeleton_to_rgb")

        qos = rclpy.qos.QoSProfile(
            depth=1,
            # durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            # reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )

        plot_model = self.declare_parameter("plot_model_output", False).value
        self.hysteresis = self.declare_parameter("hysteresis", False).value
        self.threshold = self.declare_parameter("prediction_threshold", 0.5).value
        self.lower_threshold = self.declare_parameter("hysteresis_threshold", 0.2).value
        self.user_alive_time = Duration(seconds=self.declare_parameter("user_alive_time", 1.).value)


        self.rgb_sub = Subscriber(self, Image, "/rgb/image_raw", qos_profile=qos)
        self.skeletons_sub = Subscriber(self, MarkerArrayStamped, "/body_tracking_data", qos_profile=qos)
        self.cv_bridge = cv_bridge.CvBridge()
        
        if plot_model:
            self.model_sub = Subscriber(self, ModelOutput,"model_output", qos_profile=qos)
            self.synchronizer = ApproximateTimeSynchronizer([self.rgb_sub, self.skeletons_sub, self.model_sub],
                                                            queue_size=1, slop=0.1)
        else:
            self.synchronizer = ApproximateTimeSynchronizer([self.rgb_sub, self.skeletons_sub],
                                                            queue_size=1, slop=0.1)
        self.synchronizer.registerCallback(self.topic_sync_cb)

        self.image_pub = self.create_publisher(Image, "/rgb/image_skeleton", 1)
        self.calibration = None
        self.calibration_subscriber = self.create_subscription(CameraInfo, "/depth_to_rgb/camera_info", self.calibration_cb, 1)

        self.users = {}
        self.create_timer(1., self.check_users)

        self.get_logger().info("ready")
    
    def check_users(self):
        for user_id in list(self.users.keys()):
            if self.get_clock().now() - self.users[user_id]['timer'] >= self.user_alive_time:
                del self.users[user_id]

    def calibration_cb(self, msg):
        self.get_logger().info(f"Got Calibration:\n{msg}")
        self.calibration = Calibration(msg)
        self.destroy_subscription(self.calibration_subscriber)

    def topic_sync_cb(self, rgb_msg, skeleton_msg, model_msg=None):
        if self.calibration:
            image = self.cv_bridge.imgmsg_to_cv2(rgb_msg)
            if len(skeleton_msg.markers) != 0:
                points_3d = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in skeleton_msg.markers])
                points_3d *= 1000.
                points_2d = np.squeeze(self.calibration.depth_to_rgb_image(points_3d))
                points_2d = points_2d.astype(int).clip([0, 0], image.shape[1::-1])
                for body_s in range(0, len(points_2d), 32):
                    body_id = skeleton_msg.markers[body_s].id //100
                    color = [0, 255, 0]#[(body_id*10)%255]*3
                    # color = (np.array(cm.viridis(proba))[:-1]*255) #[0, 255*label, 255*(not label)]#[(body_id*10)%255]*3
                    # color = (int(color[2]), int(color[1]), int(color[0]))
                    if model_msg:
                        if body_id in model_msg.ids:
                            idx = model_msg.ids.index(body_id)
                            probas = model_msg.probas[idx]
                            user = {"timer": self.get_clock().now(),
                                    "probas": probas,
                                    "triggered": probas>=self.threshold}
                            if body_id in self.users and self.hysteresis:
                                if self.users[body_id]["triggered"] and probas >= self.lower_threshold:
                                    user["triggered"] = True
                            self.users[body_id] = user
                            color = [0, 255*user["triggered"], 255*(not user["triggered"])]#[(body_id*10)%255]*3
                            rect = cv2.boundingRect(points_2d[body_s:body_s+32])
                            image = cv2.rectangle(image, rect, color, 3)
                            image = cv2.putText(image, f"{probas:.2f}", (rect[0]+rect[2]-80, rect[1]+40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)
                    else:
                        for body_segment in get_body_segments():
                            for i in range(len(body_segment)-1):
                                image = cv2.line(image, 
                                                points_2d[body_s + body_segment[i]], 
                                                points_2d[body_s + body_segment[i+1]], 
                                                color, 2)
                        for point in points_2d[body_s:body_s+32]:
                            image = cv2.circle(image, point, 3, color, 3)
            scale = 0.5
            width = int(image.shape[1] * scale)
            height = int(image.shape[0] * scale)
            image = cv2.resize(image, (width, height))
            self.image_pub.publish((self.cv_bridge.cv2_to_imgmsg(image, encoding=rgb_msg.encoding)))
            #cv2.imshow('skeleton_rgb', image)
            #cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonToRGB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()