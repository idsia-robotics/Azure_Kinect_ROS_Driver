import numpy as np
from cv2 import Rodrigues, projectPoints
import scipy.signal as signal
import collections
import itertools
import os

from sensor_msgs.msg import Image
from numpy.lib.stride_tricks import as_strided

name_to_dtypes = {
    "rgb8":    (np.uint8,  3),
    "rgba8":   (np.uint8,  4),
    "rgb16":   (np.uint16, 3),
    "rgba16":  (np.uint16, 4),
    "bgr8":    (np.uint8,  3),
    "bgra8":   (np.uint8,  4),
    "bgr16":   (np.uint16, 3),
    "bgra16":  (np.uint16, 4),
    "mono8":   (np.uint8,  1),
    "mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8":  (np.uint8,  1),
    "bayer_bggr8":  (np.uint8,  1),
    "bayer_gbrg8":  (np.uint8,  1),
    "bayer_grbg8":  (np.uint8,  1),
    "bayer_rggb16":     (np.uint16, 1),
    "bayer_bggr16":     (np.uint16, 1),
    "bayer_gbrg16":     (np.uint16, 1),
    "bayer_grbg16":     (np.uint16, 1),

    # OpenCV CvMat types
    "8UC1":    (np.uint8,   1),
    "8UC2":    (np.uint8,   2),
    "8UC3":    (np.uint8,   3),
    "8UC4":    (np.uint8,   4),
    "8SC1":    (np.int8,    1),
    "8SC2":    (np.int8,    2),
    "8SC3":    (np.int8,    3),
    "8SC4":    (np.int8,    4),
    "16UC1":   (np.uint16,   1),
    "16UC2":   (np.uint16,   2),
    "16UC3":   (np.uint16,   3),
    "16UC4":   (np.uint16,   4),
    "16SC1":   (np.int16,  1),
    "16SC2":   (np.int16,  2),
    "16SC3":   (np.int16,  3),
    "16SC4":   (np.int16,  4),
    "32SC1":   (np.int32,   1),
    "32SC2":   (np.int32,   2),
    "32SC3":   (np.int32,   3),
    "32SC4":   (np.int32,   4),
    "32FC1":   (np.float32, 1),
    "32FC2":   (np.float32, 2),
    "32FC3":   (np.float32, 3),
    "32FC4":   (np.float32, 4),
    "64FC1":   (np.float64, 1),
    "64FC2":   (np.float64, 2),
    "64FC3":   (np.float64, 3),
    "64FC4":   (np.float64, 4)
}

def image_to_numpy(msg):
    if not msg.encoding in name_to_dtypes:
        raise TypeError('Unrecognized encoding {}'.format(msg.encoding))

    dtype_class, channels = name_to_dtypes[msg.encoding]
    dtype = np.dtype(dtype_class)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    shape = (msg.height, msg.width, channels)

    data = np.frombuffer(msg.data, dtype=dtype).reshape(shape)
    data.strides = (
        msg.step,
        dtype.itemsize * channels,
        dtype.itemsize
    )

    if channels == 1:
        data = data[...,0]
    return data


def numpy_to_image(arr, encoding):
    if not encoding in name_to_dtypes:
        raise TypeError('Unrecognized encoding {}'.format(encoding))

    im = Image(encoding=encoding)

    # extract width, height, and channels
    dtype_class, exp_channels = name_to_dtypes[encoding]
    dtype = np.dtype(dtype_class)
    if len(arr.shape) == 2:
        im.height, im.width, channels = arr.shape + (1,)
    elif len(arr.shape) == 3:
        im.height, im.width, channels = arr.shape
    else:
        raise TypeError("Array must be two or three dimensional")

    # check type and channels
    if exp_channels != channels:
        raise TypeError("Array has {} channels, {} requires {}".format(
            channels, encoding, exp_channels
        ))
    if dtype_class != arr.dtype.type:
        raise TypeError("Array is {}, {} requires {}".format(
            arr.dtype.type, encoding, dtype_class
        ))

    # make the array contiguous in memory, as mostly required by the format
    contig = np.ascontiguousarray(arr)
    im.data = contig.tostring()
    im.step = contig.strides[0]
    im.is_bigendian = (
        arr.dtype.byteorder == '>' or
        arr.dtype.byteorder == '=' and sys.byteorder == 'big'
    )

    return im

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

from scipy.spatial.transform import Rotation as R
def get_marker_rotation(ori):
    quat = [ori.x, ori.y, ori.z, ori.w]
    rot = R.from_quat(quat).as_matrix()
    yhat = rot @ np.array([0, 1, 0])
    rot = np.arctan2(yhat[1], yhat[0])
    return rot

def angle_difference(angle1, angle2):
    return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))
