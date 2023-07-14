import rclpy.node
from rclpy.duration import Duration
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from .tf import TF

from azure_kinect_ros_msgs.msg import MarkerArrayStamped, ModelOutput
from .utils import angle_difference, PID
from robomaster_msgs.msg import LEDEffect

import numpy as np
from scipy.spatial.transform import Rotation as R
import PyKDL
import matplotlib.cm as cm


class RoboMasterDemo(rclpy.node.Node):

    def __init__(self):
        super().__init__("rm_demo")


        self.skeleton_frame = self.declare_parameter("skeleton_frame", "camera_base").value
        self.hysteresis = self.declare_parameter("hysteresis", False).value
        self.threshold = self.declare_parameter("prediction_threshold", 0.5).value
        self.lower_threshold = self.declare_parameter("hysteresis_threshold", 0.2).value
        self.user_alive_time = Duration(seconds=self.declare_parameter("user_alive_time", 1.).value)
        rm_name = self.declare_parameter("rm_name", "rm_demo").value


        self.create_subscription(Odometry, f"{rm_name}/odom", self.odom_cb, 10)
        self.vel_pub = self.create_publisher(Twist, f"{rm_name}/cmd_vel", 10)
        self.led_pub = self.create_publisher(LEDEffect, f"{rm_name}/leds/effect", 10)
        self.arm_pub = self.create_publisher(Point, f"{rm_name}/target_arm_position", 10)

        self.PID = PID(2, 0.25, 0.1)
        self.current_yaw = None
        self.yaw_zero = None

        self.skeletons_sub = Subscriber(self, MarkerArrayStamped, "/body_tracking_data", qos_profile=1)
        self.model_sub = Subscriber(self, ModelOutput,"model_output", qos_profile=1)
        self.synchronizer = ApproximateTimeSynchronizer([self.skeletons_sub, self.model_sub],
                                                        queue_size=1, slop=0.1)
        self.synchronizer.registerCallback(self.topic_sync_cb)

        self.dt = 0.1
        self.create_timer(self.dt, self.actuate_rm)

        self.tf_utils = TF(self)
        self.tform = None

        self.users = {}
        self.create_timer(1., self.check_users)

        self.get_logger().info("ready")

    def check_users(self):
        for user_id in list(self.users.keys()):
            if self.get_clock().now() - self.users[user_id]['timer'] >= self.user_alive_time:
                del self.users[user_id]

    def odom_cb(self, msg):
        ori = msg.pose.pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        yaw, _, _ = R.from_quat(quat).as_euler('zxy', degrees=False)
        if self.current_yaw is None:
            self.yaw_zero = yaw
        self.current_yaw = yaw - self.yaw_zero

    def actuate_rm(self):
        user_to_follow = None
        for user_id, user in self.users.items():
            if user["triggered"]:
                if user_to_follow is None or user_to_follow['distance']>user['distance']:
                    user_to_follow = user

        cmd = Twist()
        led_msg = LEDEffect()
        arm_msg = Point(x=0.108, z=0.067)
        if user_to_follow is not None and self.tform is not None and self.current_yaw is not None:
            pose = user_to_follow['pose']
            pos = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
            rot = PyKDL.Rotation.Quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            pos = (self.tform * PyKDL.Frame(rot, pos)).p
            pos = np.array([pos.x(), pos.y()])
            v = pos - np.array([0.40, 0.0])
            target_yaw = np.arctan2(v[1], v[0])
            
            r, g, b, _ = cm.viridis(user['probas'])
            led_msg.color = ColorRGBA(r=r, g=g, b=b, a=user['probas'])
            led_msg.effect = LEDEffect.ON
            
            cmd.angular.z = self.PID.step(angle_difference(target_yaw, self.current_yaw), self.dt)
            arm_msg.x = 0.2
        elif self.current_yaw is not None:
            cmd.angular.z = self.PID.step(angle_difference(self.yaw_zero, self.current_yaw), self.dt)
        
        self.vel_pub.publish(cmd)
        self.led_pub.publish(led_msg)
        self.arm_pub.publish(arm_msg)

    def topic_sync_cb(self, markers_msg, model_msg):
        if len(markers_msg.markers)>0:
            if self.tform is None:
                self.tform = self.tf_utils.get_transform(markers_msg.markers[0].header.frame_id, self.skeleton_frame)

            if self.tform is not None: 
                for body_s in range(0, len(markers_msg.markers), 32):
                    user_id = markers_msg.markers[body_s].id //100
                    if user_id in model_msg.ids:
                        idx = model_msg.ids.index(user_id)
                        torso_p = markers_msg.markers[body_s+1].pose
                        distance = np.hypot(torso_p.position.x, torso_p.position.y)
                        probas = model_msg.probas[idx]
                        user = {"timer": self.get_clock().now(),
                                "probas": probas,
                                "distance": distance,
                                "pose": torso_p,
                                "triggered": probas>=self.threshold}
                        if user_id in self.users and self.hysteresis:
                            if self.users[user_id]["triggered"] and probas >= self.lower_threshold:
                                user["triggered"] = True
                        self.users[user_id] = user

def main(args=None):
    rclpy.init(args=args)
    node = RoboMasterDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()