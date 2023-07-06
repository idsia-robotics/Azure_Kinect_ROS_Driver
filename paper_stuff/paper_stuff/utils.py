import numpy as np
from cv2 import Rodrigues, projectPoints
import PyKDL
from azure_kinect_ros_msgs.msg import MarkerArrayStamped
from .tf import pose_msg

# https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/opencv_compatibility/main.cpp
class Calibration:
    def __init__(self, camera_info):
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        fx = camera_info.k[0]
        fy = camera_info.k[4]

        k1 = camera_info.d[0]
        k2 = camera_info.d[1]
        k3 = camera_info.d[4]
        k4 = camera_info.d[5]
        k5 = camera_info.d[6]
        k6 = camera_info.d[7]
        codx = 0.0
        cody = 0.0
        p2 = camera_info.d[3]
        p1 = camera_info.d[2]

        self.tr = np.array([-32.0171, -1.98525, 3.88294])
        rot = np.array([[0.999968, 0.00799054, -0.000969132],
                [-0.00785875, 0.995247, 0.0970607],
                [0.00174009, -0.0970499, 0.995278]
                ])
        self.rot, _ = Rodrigues(rot)
        self.camera_matrix = np.array([[fx, 0., cx],
                        [0., fy, cy],
                        [0., 0., 1.]])
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3, k4, k5, k6])

    def depth_to_rgb_image(self, points_3d):
        points_2d, _ = projectPoints(points_3d,
                                    self.rot,
                                    self.tr,
                                    self.camera_matrix,
                                    self.dist_coeffs
                                    )
        return points_2d

def transform_skeletons(msg: MarkerArrayStamped, target_frame: str, transform: PyKDL.Frame) -> MarkerArrayStamped:
        msg.header.frame_id = target_frame
        for i in range(len(msg.markers)):
            pose = msg.markers[i].pose
            pos = PyKDL.Vector(pose.position.x, pose.position.y, pose.position.z)
            rot = PyKDL.Rotation.Quaternion(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
            msg.markers[i].pose = pose_msg(transform * PyKDL.Frame(rot, pos)).pose
            msg.markers[i].header.frame_id = target_frame
        return msg


def get_body_segments():
    spine_coords = [0, 1, 2, 3]
    left_arm_coords = [2, 4, 5, 6, 7, 8, 9]
    left_hand_coords = [7, 10]
    right_arm_coords = [2, 11, 12, 13, 14, 15, 16]
    right_hand_coords = [14, 17]
    left_leg_coords = [0, 18, 19, 20, 21]
    right_leg_coords = [0, 22, 23, 24, 25]
    head_coords = [3, 26, 27]
    face_coords = [29, 28, 27, 30, 31]
    return [spine_coords, left_arm_coords, left_hand_coords, 
            right_arm_coords, right_hand_coords, left_leg_coords, 
            right_leg_coords, head_coords, face_coords]


def markers_to_features(markers, features):
    """
    features = [jointId_[rot, xy, dist]]
    """
    sample = []
    for feat in features:
        j_id, f = feat.split("_")
        j_id = int(j_id)
        if 'xy' in f:
            sample.append(markers[j_id].pose.position.x)
            sample.append(markers[j_id].pose.position.y)
        elif 'rot' in f:
            rot = get_marker_rotation(markers[j_id].pose.orientation)
            sample.append(np.sin(rot))
            sample.append(np.cos(rot))
        elif 'dist' in f:
            d = np.linalg.norm([markers[j_id].pose.position.x, markers[j_id].pose.position.y])
            sample.append(d)
    return np.array(sample)

from scipy.spatial.transform import Rotation as R
def get_marker_rotation(ori):
    quat = [ori.x, ori.y, ori.z, ori.w]
    rot = R.from_quat(quat).as_matrix()
    yhat = rot @ np.array([0, 1, 0])
    rot = np.arctan2(yhat[1], yhat[0])
    return rot

def angle_difference(angle1, angle2):
    return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))

class PID:

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.last_e = None
        self.sum_e = 0

    def step(self, e, dt):
        if(self.last_e is not None):
            derivative = (e-self.last_e)/dt
        else:
            derivative=0
        self.last_e = e
        self.sum_e += e*dt
        return self.Kp*e + self.Kd*derivative + self.Ki*self.sum_e
