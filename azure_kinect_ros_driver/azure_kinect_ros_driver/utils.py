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
        refdata = np.load(os.path.join(file_path,"ref.npy"), allow_pickle=True)
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
        if len(self.history)>=self.smoothing_chunks:
            smoothed_dist = np.mean(np.array(self.history)[-self.smoothing_chunks:])
            if ((smoothed_dist<self.distance_threshold) and 
                (self.chunk_ix>=(self.last_detection_ix+self.cooldown_seconds*self.samplingrate/self.chunksize))):
                self.last_detection_ix = self.chunk_ix
                self.chunk_ix += 1
                return True
        self.chunk_ix += 1
        return False
    
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