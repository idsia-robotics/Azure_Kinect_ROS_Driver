import numpy as np
import scipy.signal as signal
import collections
import itertools
import os
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



import numpy as np
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