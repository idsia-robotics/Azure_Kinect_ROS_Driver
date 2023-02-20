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

class Calibration:
    def __init__(self, ak_id=2):
        if ak_id==2:
            cx = 640.689
            cy = 365.123
            fx = 606.747
            fy = 606.525
            # cx = 1025.4019775390625
            # cy = 776.4967041015625
            # fx = 970.79443359375
            # fy = 970.439697265625
            k1 = 0.3538583815097809
            k2 = -2.7078771591186523
            k3 = 1.6020128726959229
            k4 = 0.2254483550786972
            k5 = -2.500659704208374
            k6 = 1.5115599632263184
            codx = 0.0
            cody = 0.0
            p2 = 0.00015523977344855666
            p1 = 1.5893810996203683e-05

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

class Detector:
    def __init__(self, samplingrate=48000, chunksize=4800, smoothing_seconds=5, distance_threshold=5, history_seconds=60*5, cooldown_seconds=15):
        """Prepares detector. Reads reference audio from ref.npy.
        """
        assert(smoothing_seconds <= history_seconds)
        self.mfs = 100*(10**np.linspace(0,2,10)) # Log frequency ranges from 100 to 10k Hz
        self.samplingrate = samplingrate
        self.chunksize = chunksize
        self.history = collections.deque(maxlen=int(history_seconds*self.samplingrate/self.chunksize))
        file_path = os.path.dirname(os.path.realpath(__file__)) 
        refdata = np.load(os.path.join(file_path,"ref3.npy"), allow_pickle=True)
        refchunks = np.split(refdata, np.arange(self.chunksize,len(refdata),self.chunksize))
        refs = np.array([self.__process_chunk(refchunk) for refchunk in refchunks])
        self.ref = np.mean(refs, axis=0)
        self.smoothing_seconds = smoothing_seconds
        self.distance_threshold = distance_threshold
        self.history_seconds = history_seconds
        self.smoothing_chunks = int(self.smoothing_seconds*self.samplingrate/self.chunksize)
        self.cooldown_seconds = cooldown_seconds
        self.chunk_ix = 0
        self.last_detection_ix = -np.inf
        
    def consume_chunk(self, x):
        """Consumes a chunk (with shape (self.chunksize,7) and int32 datatype);
        returns True iff the signature is detected within the last smoothing_seconds.
        Needs to wait cooldown_seconds until True is returned again.
        """
        self.history.append(np.linalg.norm(self.ref - self.__process_chunk(x)))
        smoothed_dist = 100.
        if len(self.history)>=self.smoothing_chunks:
            smoothed_dist = np.mean(np.array(self.history)[-self.smoothing_chunks:])
            if ((smoothed_dist<self.distance_threshold) and 
                (self.chunk_ix>=(self.last_detection_ix+self.cooldown_seconds*self.samplingrate/self.chunksize))):
                self.last_detection_ix = self.chunk_ix
                self.chunk_ix += 1
                return True, smoothed_dist
        self.chunk_ix += 1
        return False, smoothed_dist
    
    def __process_chunk(self, x):
        """Returns spectrum for one chunk (expects (self.chunksize,7)) 
        """
        assert(x.shape == (self.chunksize,7))
        assert(x.dtype == np.int32)
        f, P = signal.periodogram(x[:,0]/(2**32), fs=self.samplingrate, scaling='spectrum', window='tukey')
        mP = []
        for f0,f1 in itertools.pairwise(self.mfs):
            mP.append(np.mean(P[(f>=f0) & (f<f1)]))
        return np.log(np.array(mP))

