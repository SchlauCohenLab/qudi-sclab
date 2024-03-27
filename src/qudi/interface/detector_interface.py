# -*- coding: utf-8 -*-

__all__ = ['DetectorInterface']

from abc import abstractmethod
from PySide2 import QtCore
from typing import Union, Type, Iterable, Mapping, Optional, Dict, List, Tuple, Sequence

from qudi.util.constraints import ScalarConstraint
from qudi.core.module import Base
from enum import Enum
import numpy as np

class StreamingMode(Enum):
    INVALID = -1
    CONTINUOUS = 0
    FINITE = 1

class SampleTiming(Enum):
    """        The sample timing can behave according to 3 different modes (Enum). Check constraints to see
        which mode is used by the hardware.

        SampleTiming.CONSTANT: The sample rate is deterministic and constant. Each sample in the stream
                               has a fixed timing determined by the sample rate.
        SampleTiming.TIMESTAMP: The sample rate is just a hint for the hardware but can not be
                                considered constant. The hardware will provide a numpy.float64 timestamp
                                in seconds from the start of the stream for each sample. This requires
                                an additional timestamp buffer array in addition to the normal channel
                                sample buffer.
        SampleTiming.RANDOM: The sample rate is just a hint for the hardware but can not be
                             considered constant. There is no deterministic time correlation between
                             samples, except that they are acquired one after another.
    """
    INVALID = -1
    CONSTANT = 0
    TIMESTAMP = 1
    RANDOM = 2

class Channel:
    """
    """
    def __init__(self, name, unit='', dtype=np.float64, streaming_mode=StreamingMode.FINITE, buffer_size=1,
                 sample_timing=SampleTiming.CONSTANT, sample_rate=None, bin_width=None):

        # attributing 'name'
        if not isinstance(name, str):
            raise TypeError('Parameter "name" must be of type str.')
        if len(name) < 1:
            raise ValueError('Parameter "name" must be non-empty str.')
        self._name = name

        # attributing 'unit'
        if not isinstance(unit, str):
            raise TypeError('Parameter "unit" must be of type str.')
        self._unit = unit

        # attributing 'dtype'
        if not isinstance(dtype, type):
            raise TypeError('Parameter "dtype" must be numpy-compatible type.')
        self._dtype = dtype

        # attributing 'sample_timing'
        if isinstance(sample_timing, SampleTiming):
            sample_timing = sample_timing.name
        if sample_timing not in SampleTiming.__members__:
            raise ValueError('{0} must be member of SampleTiming Emun class.'.format(sample_timing))
        self._sample_timing = sample_timing

        # attributing 'sample_rate'
        if not isinstance(sample_rate, ScalarConstraint) and sample_rate is not None:
            raise TypeError(
                f'"sample_rate" must be None or'
                f'{ScalarConstraint.__module__}.{ScalarConstraint.__qualname__} instance'
            )
        if sample_rate is None:
            if self._sample_timing != SampleTiming.RANDOM:
                raise ValueError('"sample_rate" ScalarConstraint must be provided if '
                                 '"sample_timing" is not SampleTiming.RANDOM')
            self._sample_rate = ScalarConstraint(default=1, bounds=(1, 1), increment=0)
        else:
            self._sample_rate = sample_rate

        # attributing 'streaming_mode'
        if isinstance(streaming_mode, StreamingMode):
            streaming_mode = streaming_mode.name
        if streaming_mode not in StreamingMode.__members__:
            raise ValueError('{0} must be member of StreamingMode Emun class.'.format(streaming_mode))
        self._streaming_mode = streaming_mode

        # attributing 'bin_width'
        if bin_width is not None:
            if not isinstance(bin_width, ScalarConstraint):
                raise TypeError(
                    f'"bin_width" must be '
                    f'{ScalarConstraint.__module__}.{ScalarConstraint.__qualname__} instance'
                )
            self._bin_width = bin_width
        else:
            self._bin_width = 1/self._sample_rate

        # attributing 'buffer_size'
        if not isinstance(buffer_size, ScalarConstraint):
            raise TypeError(
                f'"buffer_size" must be '
                f'{ScalarConstraint.__module__}.{ScalarConstraint.__qualname__} instance'
            )
        self._buffer_size = buffer_size

    def __eq__(self, other):
        if not isinstance(other, Channel):
            raise NotImplemented
        attrs = ('_name', '_unit', '_dtype', 'streaming_mode', 'buffer_size', 'sample_timing', 'sample_rate', 'bin_width')
        return all(getattr(self, a) == getattr(other, a) for a in attrs)

    @property
    def name(self):
        return self._name

    @property
    def unit(self):
        return self._unit

    @property
    def dtype(self):
        return self._dtype

    @property
    def sample_timing(self):
        return self._sample_timing

    @property
    def streaming_mode(self):
        return self._streaming_mode

    @streaming_mode.setter
    def streaming_mode(self, streaming_mode):
        if isinstance(streaming_mode, StreamingMode):
            streaming_mode = streaming_mode.name
        if streaming_mode not in StreamingMode.__members__:
            raise ValueError('{0} must be member of StreamingMode Emun class.'.format(streaming_mode))
        self._streaming_mode = streaming_mode

    @property
    def sample_rate(self):
        return self._sample_rate

    @sample_rate.setter
    def sample_rate(self, sample_rate: float):
        if not isinstance(sample_rate, ScalarConstraint) and sample_rate is not None:
            raise TypeError(
                f'"sample_rate" must be None or'
                f'{ScalarConstraint.__module__}.{ScalarConstraint.__qualname__} instance'
            )
        if sample_rate is None:
            if self._sample_timing != SampleTiming.RANDOM:
                raise ValueError('"sample_rate" ScalarConstraint must be provided if '
                                 '"sample_timing" is not SampleTiming.RANDOM')
            self._sample_rate = ScalarConstraint(default=1, bounds=(1, 1), increment=0)
        else:
            self._sample_rate = sample_rate

    @property
    def buffer_size(self):
        return self._buffer_size

    @buffer_size.setter
    def buffer_size(self, buffer_size: int):
        if not isinstance(buffer_size, ScalarConstraint):
            raise TypeError(
                f'"buffer_size" must be '
                f'{ScalarConstraint.__module__}.{ScalarConstraint.__qualname__} instance'
            )
        self._buffer_size = buffer_size

    @property
    def bin_width(self):
        return self._bin_width

    @bin_width.setter
    def bin_width(self, bin_width):
        if not isinstance(bin_width, ScalarConstraint):
            raise TypeError(
                f'"bin_width" must be '
                f'{ScalarConstraint.__module__}.{ScalarConstraint.__qualname__} instance'
            )
        self._bin_width = bin_width

    def to_dict(self):
        return {'name': self._name, 'unit': self._unit, 'dtype': self._dtype.__name__, 'streaming_mode': self._streaming_mode,
                'buffer_size': self._buffer_size, 'sample_timing': self._sample_timing, 'sample_rate': self._sample_rate, 'bin_width': self._bin_width}

    @classmethod
    def from_dict(cls, dict_repr):
        dict_repr['dtype'] = getattr(np, dict_repr['dtype'])
        return Channel(**dict_repr)

class DetectorInterface(Base):
    """ Interface for a generic input stream (finite or infinite) of data points from multiple
        channels with common data type.

        A connecting logic module can choose to manage its own buffer array(s) and let the hardware
        module read samples directly into the provided arrays for maximum data throughput using
        "read_data_into_buffer" and "read_available_data_into_buffer". Alternatively one can call
        "read_data" or "read_single_point" which will return sufficiently large data arrays that are
        newly allocated each time the method is called (less efficient but no buffer handling needed).
        In any case each time a "read_..." method is called, the samples returned are not available
        anymore and will be consumed. So multiple logic modules can not read from the same data stream.
        """

    @abstractmethod
    def get_constraints(self):
        """ Read-only property returning the constraints on the settings for this data streamer. """
        pass

    @abstractmethod
    def configure_channels(self, channels_settings):
        """ Configure a data stream. See read-only properties for information on each parameter. """
        pass

    @abstractmethod
    def get_channels(self):
        """ Configure a data stream. See read-only properties for information on each parameter. """
        pass

    @abstractmethod
    def start_measure(self):
        """ Start the fast counter. """
        pass

    @abstractmethod
    def stop_measure(self):
        """ Stop the fast counter. """
        pass

    @abstractmethod
    def get_data(self, channels=None):
        """ Polls the current timetrace data from the fast counter.

        Return value is a numpy array (dtype = int64).
        The binning, specified by calling configure() in forehand, must be
        taken care of in this hardware class. A possible overflow of the
        histogram bins must be caught here and taken care of.
        If the counter is NOT GATED it will return a tuple (1D-numpy-array, info_dict) with
            returnarray[timebin_index]
        If the counter is GATED it will return a tuple (2D-numpy-array, info_dict) with
            returnarray[gate_index, timebin_index]

        info_dict is a dictionary with keys :
            - 'elapsed_sweeps' : the elapsed number of sweeps
            - 'elapsed_time' : the elapsed time in seconds

        If the hardware does not support these features, the values should be None
        """
        pass

    @abstractmethod
    def get_status(self, channels=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        """
        pass