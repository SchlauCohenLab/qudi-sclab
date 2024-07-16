# -*- coding: utf-8 -*-

__all__ = ['TimeTagger']

import time

from qudi.interface.fast_counter_interface import FastCounterInterface
from qudi.core.statusvariable import StatusVar
import TimeTagger as tt
import numpy as np
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex

class TimeTagger(FastCounterInterface):
    """ Hardware class to controls a Time Tagger from Swabian Instruments.

    Example config for copy-paste:

    timetagger:
        module.Class: 'swabian_instruments.timetagger_fast_counter.TimeTaggerFastCounter'
        options:
            ref:
                channel: 1 # Hardware channel
                level: 500e-3 # Trigger level in V
                divider: 8 # Event divider between triggers
            apd:
                channel: 3 # Hardware channel
                level: 500e-3 # Trigger level in V
                delay: 0 # Input delay time in s


    """

    _ref = ConfigOption('ref', missing='error')
    _apd = ConfigOption('apd', missing='error')
    _sum_channels = ConfigOption('sum_channels', False, missing='warn')
    _mode = ConfigOption('mode', 0, missing='warn')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()

    def on_activate(self):
        """ Connect and configure the access to the FPGA.
        """
        self._tagger = tt.createTimeTagger()

        if 'level' in self._ref:
            self._tagger.setTriggerLevel(self._ref['channel'], self._ref['level'])

        if 'divider' in self._ref:
            self._tagger.setEventDivider(self._ref['channel'], self._ref['divider'])

        if 'level' in self._apd:
            self._tagger.setTriggerLevel(self._apd['channel'], self._apd['level'])

        if 'delay' in self._apd:
            self._tagger.setDelayHardware(self._apd['channel'], self._apd['delay'])

        self._number_of_gates = int(100)
        self._bin_width = 1
        self._record_length = int(4000)

        self.statusvar = 0

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
        if self.module_state() == 'locked':
            self._stream.stop()
        self._stream.clear()
        self._stream = None

    # ================ Fast counter interface ===================

    def configure(self, bin_width_s, record_length_s, number_of_gates=1):

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

        self._stream = tt.TimeDifferences(
            tagger=self._tagger,
            click_channel=self._apd['channel'],
            start_channel=self._ref['channel'],
            next_channel=self._ref['channel'],
            sync_channel=tt.CHANNEL_UNUSED,
            binwidth=int(np.round(self._bin_width * 1e12)),
            n_bins=int(self._record_length),
            n_histograms=number_of_gates)

        self._stream.stop()

        return bin_width_s, record_length_s, number_of_gates

    def start_measure(self):
        """ Start the fast counter. """
        self.module_state.lock()

        self._stream.clear()
        self._stream.start()

        self.statusvar = 2

    def stop_measure(self):
        """ Stop the fast counter. """
        if self.module_state() == 'locked':
            self._stream.stop()
            self.module_state.unlock()
        self.statusvar = 1

    def pause_measure(self):
        """ Pauses the current measurement.

        Fast counter must be initially in the run state to make it pause.
        """
        if self.module_state() == 'locked':
            self._stream.stop()
            self.statusvar = 3

    def continue_measure(self):
        """ Continues the current measurement.

        If fast counter is in pause state, then fast counter will be continued.
        """
        if self.module_state() == 'locked':
            self._stream.start()
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
        return np.array(self._stream.getData(), dtype='int64'), info_dict

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

    def start_tt_stream(self):
        """ Start the time-tag streaming measurement of the fast counter."""
        self._stream = tt.TimeTagStream(self._tagger, 1e9, [self._ref['channel'], self._apd['channel']])

    def get_buffer_data(self):
        """ Receives the current timetrace data from the fast counter."""
        data = self._stream.getData()
        return data

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
