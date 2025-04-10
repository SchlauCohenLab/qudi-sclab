# -*- coding: utf-8 -*-

__all__ = ['TimeTagger']

import time
from datetime import datetime

from qudi.interface.fast_counter_interface import FastCounterInterface
from qudi.core.statusvariable import StatusVar
import TimeTagger as tt
import numpy as np
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex, RecursiveMutex
from PySide2 import QtCore
from collections import deque
import numba

class TimeDifferenceList2ChanContinuous(tt.CustomMeasurement):
    """
    Custom Measurement for QKD with a time-bin protocol
    Internal note: ticket 438
    Author: Michael Schlagmüller
    Date: October 2021

    Sync signal (start) to one channel and the quantum signals to the other (click),
    so that the timestamps restart from zero every time there is a detection on the start channel.

    multiple start-multiple stop

    1 start channel, 2 stop channels

    Similar to TimeDifference but instead of a Histogram the time differences are returned.

    Example

    data = measurement.getData()

    Stop-clicks: 748408 # len(data)
    print(datA)
    # t of start (ps)    td to stop   channel number of stop signal
    [[1352061357154          1716             2]
     [1352062467282       1111844             3]
     [1352064681694       3326256             2]
     ...
     [2352059749989       1123944             2]
     [2352060867665       2241620             3]
     [2352063089739       4463694             2]]

    sleep(...)

    data = measurement.getData()
    Stop-clicks: 739853 # len(data)
    # t of start (ps)    td to stop   channel number of stop signal
    [[2465715809473       4505242             2]
     [2465715809574       4505343             3]
     [2465718065512       1127294             3]
     ...
     [3465711889335       4518677             2]
     [3465711889422       4518764             3]
     [3465714132032       1121347             3]]

    Parameters

    interval:     Data is returned as a list and every a list entry is completed when the 'interval' time has passed.
    buffer_size:  buffer size of the internal buffer used
    """

    def __init__(self, tagger, click_channel1, click_channel2, start_channel, interval, buffer_size):
        tt.CustomMeasurement.__init__(self, tagger)
        assert click_channel1 != start_channel
        assert click_channel2 != start_channel
        assert click_channel1 != click_channel2
        buffer_size = int(buffer_size)
        interval = int(interval)
        assert buffer_size > 0
        assert interval >= 1e11, "The interval should not bee to short, at least 0.1s == 1e11 ps"
        self.click_channel1 = click_channel1
        self.click_channel2 = click_channel2
        self.start_channel = start_channel
        self.interval = interval
        # we increase the buffer size by one, because we always leave one row empty
        buffer_size = buffer_size + 1
        self.buffer_size = buffer_size
        self.data = np.zeros([buffer_size, 3], dtype=np.int64)
        # we need a list for returning the added block boundaries
        # in case the hard coded buffer size of 10000 is exceeded, an exception is thrown
        self.interval_new_start_indices = np.zeros(10000, dtype=np.int32)

        # The method register_channel(channel) activates
        # that data from the respective channels is transferred
        # from the Time Tagger to the PC.
        self.register_channel(channel=click_channel1)
        self.register_channel(channel=click_channel2)
        self.register_channel(channel=start_channel)

        self.clear_impl()

        # At the end of a CustomMeasurement construction,
        # we must indicate that we have finished.
        self.finalize_init()

    def __del__(self):
        # The measurement must be stopped before deconstruction to avoid
        # concurrent process() calls.
        self.stop()

    def getData(self):
        # Acquire a lock this instance to guarantee that process() is not running in parallel
        # This ensures to return a consistent data.
        self._lock()
        if (self.read_index == self.write_index):
            # we have not stored anything - return an empty array
            self._unlock()
            return np.zeros([0, 3], dtype=np.int64)
        else:
            # valid state - data is stored
            buffered_time = abs(self.__getBufferedTime())
            if buffered_time < self.interval:
                # check integrity - should never fail
                if len(self.interval_boundary_list) > 0:
                    self._unlock()
                    assert False, "len(self.interval_boundary_list) == 0"
                print("Internal buffered time {:.2e} is less that the interval defined at construction time {:.2e}. Please check with has_new_data() whether data is ready to be returned via .getData()".format(
                    buffered_time, self.interval))
                self._unlock()
                return np.zeros([0, 3], dtype=np.int64)

            # check integrity - should never fail
            if len(self.interval_boundary_list) == 0:
                self._unlock()
                assert False, "len(self.interval_boundary_list) > 0"
            next_interval_start = self.interval_boundary_list.pop()
            if next_interval_start >= self.read_index:
                # no wrap read case
                arr = self.data[self.read_index:next_interval_start, :].copy()
            else:
                # wrap read case
                # concatenate does a copy - so no explicit copy required
                arr = np.concatenate((self.data[self.read_index:self.buffer_size, :], self.data[0:next_interval_start, :]))
            self.read_index = next_interval_start
            # We have gathered the data, unlock, so measuring can continue.
        self._unlock()
        return arr

    def getBufferedTime(self):
        self._lock()
        buffered_time = self.__getBufferedTime()
        self._unlock()
        return buffered_time

    def __getBufferedTime(self):
        if self.read_index == self.write_index:
            buffered_time = 0
        else:
            buffered_time = self.data[(self.write_index-1), 0] - (self.init_time + (self.interval_counter-len(self.interval_boundary_list)) * self.interval)
        return buffered_time

    def has_new_data(self):
        self._lock()
        if len(self.interval_boundary_list) > 0:
            if not (self.__getBufferedTime() >= self.interval):
                self._unlock()
                assert False, "bufferedTime {} must be greater or equal interval {}".format(self.__getBufferedTime(), self.interval)
            result = True
        else:
            if not (self.__getBufferedTime() < self.interval):
                self._unlock()
                assert False, "self.__getBufferedTime() {} < self.interval {}".format(self.__getBufferedTime(), self.interval)
            result = False
        self._unlock()
        return result

    def clear_impl(self):
        # The lock is already acquired within the backend.
        self.start_channel_timestamp = 0
        self.init_time = 0
        self.write_index = 0
        self.read_index = 0
        self.interval_boundary_list = deque()
        self.interval_counter = 0

    def on_start(self):
        # The lock is already acquired within the backend.
        self.clear_impl()

    def on_stop(self):
        # The lock is already acquired within the backend.
        pass

    @staticmethod
    @numba.jit(nopython=True, nogil=True)
    def fast_process(
            tags,
            data,
            click_channel1,
            click_channel2,
            start_channel,
            start_channel_timestamp,
            init_time,
            write_index,
            read_index,
            interval,
            interval_counter,
            interval_new_start_indices,
    ):
        """
        A precompiled version of the algorithm for better performance
        nopython=True: Only a subset of the python syntax is supported.
                       Avoid everything but primitives and numpy arrays.
                       All slow operations will yield an exception
        nogil=True:    This method will release the global interpreter lock. So
                       this method can run in parallel with other python code.
        """
        _buffer_size = len(data)
        new_start_indices_buffer_size = len(interval_new_start_indices)
        finished_intervals = 0
        for tag in tags:
            # tag.type can be: 0 - TimeTag, 1- Error, 2 - OverflowBegin, 3 -
            # OverflowEnd, 4 - MissedEvents (you can use the TimeTagger.TagType IntEnum)
            if tag['type'] != tt.TagType.TimeTag:
                # tag is not a TimeTag, so we are in an error state, e.g. overflow
                # we only do simply overflow-handling, do not use this method with overflows
                # do the same as clear_impl()
                print("Time Tagger hardware Buffer overflow - reset local buffer")
                return -1, -1, -1, -1, -1
            else:
                if init_time == 0:
                    # when we did not call get data before - take the current time as the last get data call
                    init_time = tag['time']

                if (tag['channel'] == click_channel1 or tag['channel'] == click_channel2) and start_channel_timestamp != 0:
                    # check whether the buffer is exceeded
                    if (write_index + 1 == read_index) or (write_index == _buffer_size - 1 and read_index == 0):
                        print("Internal buffer exceeded. Please use a bigger buffer or call .getData() faster.")
                        break
                    # write a new timestamp
                    data[write_index, :] = [tag['time'] * 1E-12, (tag['time'] - start_channel_timestamp) * 1E-12, tag['channel']]
                    write_index = write_index + 1
                    if write_index == _buffer_size:
                        write_index = 0

                    # did we cross the interval boundary
                    while tag['time'] >= init_time + interval * (interval_counter + 1):
                        assert finished_intervals < new_start_indices_buffer_size, "Error: buffer for boundaries within fast_process exceeded."
                        interval_new_start_indices[finished_intervals] = write_index
                        interval_counter += 1
                        finished_intervals += 1

                elif tag['channel'] == start_channel:
                    # found a new start timestamp
                    start_channel_timestamp = tag['time']

        return start_channel_timestamp, init_time, write_index, read_index, interval_counter

    def process(self, incoming_tags, begin_time, end_time):
        """
        Main processing method for the incoming raw time-tags.

        The lock is already acquired within the backend.
        self.data is provided as reference, so it must not be accessed
        anywhere else without locking the mutex.

        Parameters
        ----------
        incoming_tags
            The incoming raw time tag stream provided as a read-only reference.
            The storage will be deallocated after this call, so you must not store a reference to
            this object. Make a copy instead.
            Please note that the time tag stream of all channels is passed to the process method,
            not only the ones from register_channel(...).
        begin_time
            Begin timestamp of the current data block.
        end_time
            End timestamp of the current data block.
        """
        interval_counter_before = self.interval_counter
        self.start_channel_timestamp, self.init_time, self.write_index, self.read_index, self.interval_counter = TimeDifferenceList2ChanContinuous.fast_process(
            incoming_tags,
            self.data,
            self.click_channel1,
            self.click_channel2,
            self.start_channel,
            self.start_channel_timestamp,
            self.init_time,
            self.write_index,
            self.read_index,
            self.interval,
            self.interval_counter,
            self.interval_new_start_indices,
        )
        # -1: error state returned from fast_process - reset the measurement class
        if self.start_channel_timestamp == -1:
            self.clear_impl()
        else:
            new_intervals = self.interval_counter - interval_counter_before
            # add the new interval boundaries
            for i in range(new_intervals):
                self.interval_boundary_list.appendleft(self.interval_new_start_indices[i])


class TimeTagger(FastCounterInterface):
    """ Hardware class to controls a Time Tagger from Swabian Instruments.

    Example config for copy-paste:

    timetagger:
        module.Class: 'swabian.timetagger.TimeTagger'
        options:
            ref_channel:
                channel: 1 # Hardware channel
                level: 500e-3 # Trigger level in V
                divider: 8 # Event divider between triggers
                delay: 5e-9 # Input delay time in s
            apd_channels:
                apd_0:
                    channel: 2 # Hardware channel
                    level: 500e-3 # Trigger level in V
                    delay: 5e-9 # Input delay time in s
                apd_1:
                    channel: 3 # Hardware channel
                    level: 500e-3 # Trigger level in V
                    delay: 5e-9 # Input delay time in s


    """

    _ref_channel = ConfigOption('ref_channel', missing='error')
    _apd_channels = ConfigOption('apd_channels', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._mutex = Mutex()
        self._thread_lock = RecursiveMutex()

        self._read_stream_timer = None

    def on_activate(self):
        """ Connect and configure the access to the FPGA.
        """

        self._apd_channels_index = [ ch['channel'] for ch in self._apd_channels.values() ]
        self._tagger = tt.createTimeTagger()
        self._tagger.reset()
        self._tagger.setConditionalFilter(trigger=[self._ref_channel['channel']], filtered=self._apd_channels_index)
        #self._tagger.setConditionalFilter(trigger=self._apd_channels_index, filtered=[self._ref_channel['channel']])

        #self._tagger.setTimeTaggerChannelNumberScheme(tt.TT_CHANNEL_NUMBER_SCHEME_ONE)

        #sleep(10)

        if 'level' in self._ref_channel:
            self._tagger.setTriggerLevel(self._ref_channel['channel'], self._ref_channel['level'])

        if 'divider' in self._ref_channel:
            self._tagger.setEventDivider(self._ref_channel['channel'], self._ref_channel['divider'])

        for apd_ch in self._apd_channels.values():
            if 'level' in apd_ch:
                self._tagger.setTriggerLevel(apd_ch['channel'], apd_ch['level'])

            if 'delay' in apd_ch:
                self._tagger.setDelaySoftware(apd_ch['channel'], int(apd_ch['delay']*1e12))

        self._tagger.sync()

        self._number_of_gates = int(100)
        self._bin_width = 1
        self._record_length = int(4000)
        self.n_events = 10e6
        self._interval = 1e12 # duration in ps for each returned block with .getData()
        self._stream_data = None

        self.statusvar = 0

        self._stream = tt.TimeTagStream(tagger=self._tagger,
                        n_max_events=self.n_events,
                        channels=[self._ref_channel['channel']]+self._apd_channels_index)
        #self._tdl = TimeDifferenceList2ChanContinuous(self._tagger, self._apd_channels_index[0], self._apd_channels_index[1], self._ref_channel['channel'], self._interval, self.n_events)

        self._read_stream_timer = QtCore.QTimer()
        self._read_stream_timer.setSingleShot(True)
        self._read_stream_timer.timeout.connect(self.read_stream, QtCore.Qt.QueuedConnection)

    def reset(self):
        """ Reset the hardware to its initial state."""
        self._tagger.reset()

    def get_constraints(self):
        """ Retrieve the hardware constrains from the Fast counting device.

        @return dict: dict with keys being the constraint names as string and
                      items are the definition for the constaints.

         The keys of the returned dictionary are the str name for the constraints
        (which are set in this method).

                    NO OTHER KEYS SHOULD BE INVENTED!

        If you are not sure about the meaning, look in other hardware files to
        get an impression. If still additional constraints are needed, then they
        have to be added to all files containing this interface.

        The items of the keys are again dictionaries which have the generic
        dictionary form:
            {'min': <value>,
             'max': <value>,
             'step': <value>,
             'unit': '<value>'}

        Only the key 'hardware_binwidth_list' differs, since they
        contain the list of possible binwidths.

        If the constraints cannot be set in the fast counting hardware then
        write just zero to each key of the generic dicts.
        Note that there is a difference between float input (0.0) and
        integer input (0), because some logic modules might rely on that
        distinction.

        ALL THE PRESENT KEYS OF THE CONSTRAINTS DICT MUST BE ASSIGNED!
        """

        constraints = dict()

        # the unit of those entries are seconds per bin. In order to get the
        # current binwidth in seonds use the get_binwidth method.
        constraints['hardware_binwidth_list'] = [1 / 1000e6]

        # TODO: think maybe about a software_binwidth_list, which will
        #      postprocess the obtained counts. These bins must be integer
        #      multiples of the current hardware_binwidth

        return constraints

    def on_deactivate(self):
        """ Deactivate the FPGA.
        """

        self._read_stream_timer.stop()
        self._read_stream_timer.timeout.disconnect()

        if self.module_state() == 'locked':
            self.stop_stream()

    # ================ Fast counter interface ===================

    def configure(self, bin_width_s, record_length_s, number_of_gates=1, apd_ch=None):

        """ Configuration of the fast counter.

        @param float bin_width_s: Length of a single time bin in the time trace
                                  histogram in seconds.
        @param float record_length_s: Total length of the timetrace/each single
                                      gate in seconds.
        @param int number_of_gates: optional, number of gates in the pulse
                                    sequence. Ignore for not gated counter.

        @return tuple(binwidth_s, gate_length_s, number_of_gates):
                    binwidth_s: float the actual set binwidth in seconds
                    gate_length_s: the actual set gate length in seconds
                    number_of_gates: the number of gated, which are accepted
        """
        self._number_of_gates = number_of_gates
        self._bin_width = bin_width_s
        self._record_length = 1 + int(record_length_s / bin_width_s)
        self.statusvar = 1
        if apd_ch is None:
            apd_ch = list(self._apd_channels.keys())[0]
        click_ch = self._apd_channels[apd_ch]

        self._meas = tt.TimeDifferences(
            tagger=self._tagger,
            click_channel=click_ch['channel'],
            start_channel=self._ref_channel['channel'],
            next_channel=self._ref_channel['channel'],
            sync_channel=tt.CHANNEL_UNUSED,
            binwidth=int(self._bin_width),
            n_bins=int(self._record_length),
            n_histograms=number_of_gates
            )

        self._meas.stop()

        return bin_width_s, record_length_s, number_of_gates

    def start_measure(self):
        """ Start the fast counter. """
        if self._meas:
            self.module_state.lock()

            self._meas.start()

            self.statusvar = 2
        else:
            self.log.warning('Measurement need to be configure before start measuring')

    def stop_measure(self):
        """ Stop the fast counter. """
        if self.module_state() == 'locked':
            self._meas.stop()
            self.module_state.unlock()
        self.statusvar = 1

    def pause_measure(self):
        """ Pauses the current measurement.

        Fast counter must be initially in the run state to make it pause.
        """
        if self.module_state() == 'locked':
            self._meas.stop()
            self.statusvar = 3

    def continue_measure(self):
        """ Continues the current measurement.

        If fast counter is in pause state, then fast counter will be continued.
        """
        if self.module_state() == 'locked':
            self._meas.start()
            self.statusvar = 2

    def is_gated(self):
        """ Check the gated counting possibility.

        Boolean return value indicates if the fast counter is a gated counter
        (TRUE) or not (FALSE).
        """
        return True

    def get_data_trace(self):
        """ Polls the current timetrace data from the fast counter.

        @return numpy.array: 2 dimensional array of dtype = int64. This counter
                             is gated the the return array has the following
                             shape:
                                returnarray[gate_index, timebin_index]

        The binning, specified by calling configure() in forehand, must be taken
        care of in this hardware class. A possible overflow of the histogram
        bins must be caught here and taken care of.
        """
        info_dict = {'elapsed_sweeps': None,
                     'elapsed_time': None}  # TODO : implement that according to hardware capabilities
        return np.array(self._meas.getData(), dtype='int64'), info_dict

    def get_status(self):
        """ Receives the current status of the Fast Counter and outputs it as
            return value.

        0 = unconfigured
        1 = idle
        2 = running
        3 = paused
        -1 = error state
        """
        return self.statusvar

    def get_binwidth(self):
        """ Returns the width of a single timebin in the timetrace in seconds. """
        width_in_seconds = self._bin_width * 1e-9
        return width_in_seconds

    # ================ Time-tag streaming ===================

    def start_stream(self):
        """ Start the time-tag streaming measurement of the fast counter."""

        with self._thread_lock:
            #self._tdl.clear_impl()
            self._stream.start()
            self.module_state.lock()
            self._stream_data = []
            self.__start_timer()

    def stop_stream(self):
        """ Start the time-tag streaming measurement of the fast counter."""

        self.module_state.unlock()

        self.read_stream()

        self._stream.stop()

        return

    def read_stream(self):
        """ Start the time-tag streaming measurement of the fast counter."""
        
        with self._thread_lock:

            data = self._stream.getData()
            channels = data.getChannels()           # The channel numbers
            timestamps = data.getTimestamps()       # The timestamps in ps
            overflow_types = data.getEventTypes()   # TimeTag = 0, Error = 1, OverflowBegin = 2, OverflowEnd = 3, MissedEvents = 4
            missed_events = data.getMissedEvents()  # The numbers of missed events in case of overflow

            timestamps = timestamps[overflow_types==0]

            # Use boolean indexing to filter channels
            apd_indices = np.where(np.isin(channels, self._apd_channels_index))[0]

            # Calculate macro_time and micro_time using vectorized operations
            tags_ch = channels[apd_indices]
            tags_ts = timestamps[apd_indices] * 1E-12
            tags_dt = (timestamps[apd_indices] - timestamps[apd_indices - 1]) * 1E-12

            if len(self._stream_data) == 0:
                self._stream_data = np.array([tags_ch, tags_ts, tags_dt])
            else:
                self._stream_data = np.hstack((self._stream_data, np.array([tags_ch, tags_ts, tags_dt])))

            if self.module_state() == 'locked':
                self.__start_timer()
            
            return
        
    def __start_timer(self):
        time.sleep(0.2)
        if self.thread() is not QtCore.QThread.currentThread():
            QtCore.QMetaObject.invokeMethod(self._read_stream_timer,
                                            'start',
                                            QtCore.Qt.BlockingQueuedConnection)
        else:
            self._read_stream_timer.start()

    def get_stream_data(self):
        """ Start the time-tag streaming measurement of the fast counter."""

        return self._stream_data

    # ================ Slow counter interface ===================

    def set_up_clock(self, clock_frequency=None, clock_channel=None):
        """ Configures the hardware clock of the TimeTagger for timing

        @param float clock_frequency: if defined, this sets the frequency of
                                      the clock
        @param string clock_channel: if defined, this is the physical channel
                                     of the clock

        @return int: error code (0:OK, -1:error)
        """

        self._count_frequency = clock_frequency
        return 0

    def set_up_counter(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       counter_buffer=None):
        """ Configures the actual counter with a given clock.

        @param str counter_channel: optional, physical channel of the counter
        @param str photon_source: optional, physical channel where the photons
                                  are to count from
        @param str counter_channel2: optional, physical channel of the counter 2
        @param str photon_source2: optional, second physical channel where the
                                   photons are to count from
        @param str clock_channel: optional, specifies the clock channel for the
                                  counter
        @param int counter_buffer: optional, a buffer of specified integer
                                   length, where in each bin the count numbers
                                   are saved.

        @return int: error code (0:OK, -1:error)
        """

        # currently, parameters passed to this function are ignored -- the channels used and clock frequency are
        # set at startup
        if self._mode == 1:
            channel_combined = tt.Combiner(self._tagger, channels = [self._channel_apd_0, self._channel_apd_1])
            self._channel_apd = channel_combined.getChannel()

            self.counter = tt.Counter(
                self._tagger,
                channels=[self._channel_apd],
                binwidth=int((1 / self._count_frequency) * 1e12),
                n_values=1
            )
        elif self._mode == 2:
            self.counter0 = tt.Counter(
                self._tagger,
                channels=[self._channel_apd_0],
                binwidth=int((1 / self._count_frequency) * 1e12),
                n_values=1
            )

            self.counter1 = tt.Counter(
                self._tagger,
                channels=[self._channel_apd_1],
                binwidth=int((1 / self._count_frequency) * 1e12),
                n_values=1
            )
        else:
            self._channel_apd = self._channel_apd_0
            self.counter = tt.Counter(
                self._tagger,
                channels=[self._channel_apd],
                binwidth=int((1 / self._count_frequency) * 1e12),
                n_values=1
            )

        self.log.info('set up counter with {0}'.format(self._count_frequency))
        return 0

    def get_counter_channels(self):
        if self._mode < 2:
            return self._channel_apd
        else:
            return [self._channel_apd_0, self._channel_apd_1]

    def get_constraints(self):
        """ Get hardware limits the device

        @return SlowCounterConstraints: constraints class for slow counter

        FIXME: ask hardware for limits when module is loaded
        """
        constraints = SlowCounterConstraints()
        constraints.max_detectors = 2
        constraints.min_count_frequency = 1e-3
        constraints.max_count_frequency = 10e9
        constraints.counting_mode = [CountingMode.CONTINUOUS]
        return constraints

    def get_counter(self, samples=None):
        """ Returns the current counts per second of the counter.

        @param int samples: if defined, number of samples to read in one go

        @return numpy.array(uint32): the photon counts per second
        """

        time.sleep(2 / self._count_frequency)
        if self._mode < 2:
            return self.counter.getData() * self._count_frequency
        else:
            return np.array([self.counter0.getData() * self._count_frequency,
                             self.counter1.getData() * self._count_frequency])

    def close_counter(self):
        """ Closes the counter and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        self._tagger.reset()
        return 0

    def close_clock(self):
        """ Closes the clock and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        return 0
