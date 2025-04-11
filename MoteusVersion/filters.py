from scipy import signal
import numpy as np
import collections


class Filter(object):
    '''Parent class for filters, to help with type hinting. 

    Note: for filter modularity, all child classes should have a filter() 
    function that takes only the most recent value. This way, different filters 
    can be passed to objects constructors interchangeably'''

    def filter(self, new_val):
        raise ValueError('filter() not implemented for child class of Filter')


class PassThroughFilter(Filter):
    def filter(self, new_val):
        return new_val


class Butterworth():
    '''Implements a real-time Butterworth filter using second orded cascaded filters.'''

    def __init__(self, N: int, Wn: float, btype='low', fs=None):
        ''' 
        N: order
        Wn: (default) normalized cutoff freq (cutoff freq / Nyquist freq). If fs is passed, cutoff is in Hz.
        btyple: 'low', 'high', or 'bandpass'
        fs: Optional: sample freq, Hz. If not None, Wn describes the cutoff freq in Hz
        '''
        self.N = N
        self.fs = fs
        self.Wn = Wn
        self.btype = btype
        if self.fs is not None:
            self._Wn = self.Wn/(self.fs/2)
        else:
            self._Wn = Wn
        self.sos = signal.butter(N=self.N, Wn=self._Wn,
                                 btype=self.btype, output='sos')
        self.zi = signal.sosfilt_zi(self.sos)
        self.first_value = True

    def filter(self, new_val: float) -> float:
        if self.first_value:
            self.zi = self.zi*new_val
            self.first_value = False
        filtered_val, self.zi = signal.sosfilt(
            sos=self.sos, x=[new_val], zi=self.zi)
        return filtered_val[0]

    def restart(self):
        self.__init__(N=self.N, Wn=self._Wn)


class MovingAverage(Filter):
    '''Implements a real-time moving average filter.'''

    def __init__(self, window_size):
        self.window_size = window_size
        self.deque = collections.deque([], maxlen=window_size)

    def filter(self, new_val):
        # TODO: Optimize for efficiency if window size is large
        self.deque.append(new_val)
        return np.mean(self.deque)
    
    def restart(self):
        self.__init__(window_size=self.window_size) 
