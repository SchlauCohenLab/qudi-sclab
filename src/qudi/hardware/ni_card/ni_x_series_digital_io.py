# -*- coding: utf-8 -*-

"""
This file contains the qudi hardware module to use a National Instruments X-series card as mixed
signal input data streamer.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

import ctypes
import time
import numpy as np
import nidaqmx as ni
from functools import wraps
from typing import Tuple, List, Optional, Sequence, Union
from nidaqmx._lib import lib_importer  # Due to NIDAQmx C-API bug needed to bypass property getter
from nidaqmx.stream_readers import CounterReader
from nidaqmx.stream_readers import AnalogMultiChannelReader as _AnalogMultiChannelReader
from nidaqmx.constants import FillMode, READ_ALL_AVAILABLE
try:
    from nidaqmx._task_modules.read_functions import _read_analog_f_64
except ImportError:
    pass

from qudi.interface.switch_interface import SwitchInterface
from qudi.core.configoption import ConfigOption
from qudi.util.helpers import natural_sort
from qudi.util.constraints import ScalarConstraint

class NIXSeriesDigitalIO(DataInStreamInterface, DetectorInterface):
    """
    A National Instruments device that can detect and count digital pulses and measure analog
    voltages as data stream.

    !!!!!! NI USB 63XX, NI PCIe 63XX and NI PXIe 63XX DEVICES ONLY !!!!!!

    See [National Instruments X Series Documentation](@ref nidaq-x-series) for details.

    Example config for copy-paste:

    nicard_6343_digital_io:
        module.Class: 'ni_card.ni_x_series_digital_io.NIXSeriesDigitalIO'
        options:
            device_name: 'Dev1'
            digital_sources:  # optional
                'acceptor': 'PFI0'
                'donor': 'PFI12'
    """

    # config options
    _device_name = ConfigOption(name='device_name', default='Dev1', missing='warn')
    _digital_sources = ConfigOption(name='digital_sources', default=dict(), missing='info')
    _analog_sources = ConfigOption(name='analog_sources', default=dict(), missing='info')
    _external_sample_clock_source = ConfigOption(name='external_sample_clock_source',
                                                 default=None,
                                                 missing='nothing')
    _external_sample_clock_frequency = ConfigOption(
        name='external_sample_clock_frequency',
        default=None,
        missing='nothing',
        constructor=lambda x: x if x is None else float(x)
    )
    _adc_voltage_range = ConfigOption('adc_voltage_range', default=(-10, 10), missing='info')
    _max_channel_samples_buffer = ConfigOption(name='max_channel_samples_buffer',
                                               default=1024**2,
                                               missing='info',
                                               constructor=lambda x: max(int(round(x)), 1024**2))
    _rw_timeout = ConfigOption('read_write_timeout', default=10, missing='nothing')
    _clock_counter = ConfigOption('clock_counter', default=None, missing='nothing')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # NIDAQmx device handle
        self._device_handle = None
        # Task handles for NIDAQmx tasks
        self._di_task_handles = list()
        self._ai_task_handle = None
        self._clk_task_handle = None
        # nidaqmx stream reader instances to help with data acquisition
        self._di_readers = list()
        self._ai_reader = None
        # Internal settings
        self.__sample_rate = -1
        self.__buffer_size = -1
        self.__streaming_mode = None
        # List of all available counters and terminals for this device
        self.__all_counters = tuple()
        self.__all_digital_terminals = tuple()
        self.__all_analog_terminals = tuple()
        # currently active channels
        self.__active_channels = tuple()
        # Stored hardware constraints
        self._constraints = None
        self.__tmp_buffer = None

    def on_activate(self):
        """
        Starts up the NI-card and performs sanity checks.
        """
        # Check if device is connected and set device to use
        dev_names = ni.system.System().devices.device_names
        if self._device_name.lower() not in set(dev.lower() for dev in dev_names):
            raise ValueError(
                f'Device name "{self._device_name}" not found in list of connected devices: '
                f'{dev_names}\nActivation of NIXSeriesInStreamer failed!'
            )
        for dev in dev_names:
            if dev.lower() == self._device_name.lower():
                self._device_name = dev
                break
        self._device_handle = ni.system.Device(self._device_name)

        self.__all_counters = tuple(
            ctr.split('/')[-1] for ctr in self._device_handle.co_physical_chans.channel_names if
            'ctr' in ctr.lower()
        )
        self.__all_digital_terminals = tuple(
            term.rsplit('/', 1)[-1].lower() for term in self._device_handle.terminals if
            'PFI' in term
        )
        self.__all_analog_terminals = tuple(
            term.rsplit('/', 1)[-1].lower() for term in
            self._device_handle.ai_physical_chans.channel_names
        )

        # Check digital input terminals
        if self._digital_sources:
            source_set = set(self._extract_terminal(src) for src in self._digital_sources.values())
            invalid_sources = source_set.difference(set(self.__all_digital_terminals))
            if invalid_sources:
                self.log.error(
                    'Invalid digital source terminals encountered. Following sources will '
                    'be ignored:\n  {0}\nValid digital input terminals are:\n  {1}'
                    ''.format(', '.join(natural_sort(invalid_sources)),
                              ', '.join(self.__all_digital_terminals)))
            valid_digital_src = natural_sort(source_set.difference(invalid_sources))
            d_src = self._digital_sources.copy()
            for ch, src in d_src.items():
                if src not in valid_digital_src:
                    self._digital_sources.pop(ch)

                # Check analog input channels
        if self._analog_sources:
            source_set = set(self._extract_terminal(src) for src in self._analog_sources.values())
            invalid_sources = source_set.difference(set(self.__all_analog_terminals))
            if invalid_sources:
                self.log.error('Invalid analog source channels encountered. Following sources will '
                               'be ignored:\n  {0}\nValid analog input channels are:\n  {1}'
                               ''.format(', '.join(natural_sort(invalid_sources)),
                                         ', '.join(self.__all_analog_terminals)))
            valid_analog_src = natural_sort(source_set.difference(invalid_sources))
            a_src = self._analog_sources.copy()
            for ch, src in a_src.items():
                if src not in valid_analog_src:
                    self._analog_sources.pop(ch)

        # Check if all input channels fit in the device
        if len(self._digital_sources) > 3:
            raise ValueError(
                'Too many digital channels specified. Maximum number of digital channels is 3.'
            )
        if len(self._analog_sources) > 16:
            raise ValueError(
                'Too many analog channels specified. Maximum number of analog channels is 16.'
            )

        # Check if there are any valid input channels left
        if not self._analog_sources and not self._digital_sources:
            raise ValueError(
                'No valid analog or digital sources defined in config. Activation of '
                'NIXSeriesInStreamer failed!'
            )

        # Create constraints
        channel_units = {chnl: 'counts/s' for chnl in self._digital_sources.keys()}
        channel_units.update({chnl: 'V' for chnl in self._analog_sources.keys()})
        self._constraints = DataInStreamConstraints(
            channel_units=channel_units,
            sample_timing=SampleTiming.CONSTANT,
            streaming_modes=[StreamingMode.CONTINUOUS], # TODO: Implement FINITE streaming mode
            data_type=np.float64,
            channel_buffer_size=ScalarConstraint(default=1024**2,
                                                 bounds=(2, self._max_channel_samples_buffer),
                                                 increment=1,
                                                 enforce_int=True),
            # FIXME: What is the minimum frequency for the digital counter timebase?
            sample_rate=ScalarConstraint(default=50.0,
                                         bounds=(self._device_handle.ai_min_rate,
                                                 self._device_handle.ai_max_multi_chan_rate),
                                         increment=1,
                                         enforce_int=False)
        )

        self._channels = dict()
        for ch, src in self._digital_sources.items():
            self._channels[ch] = Channel(ch, unit=channel_units[ch],
                                     dtype=np.float64,
                                     streaming_mode=StreamingMode.CONTINUOUS, buffer_size=ScalarConstraint(default=1024**2,
                                                 bounds=(2, self._max_channel_samples_buffer),
                                                 increment=1,
                                                 enforce_int=True),
                                     sample_timing=SampleTiming.CONSTANT, sample_rate=ScalarConstraint(default=50.0,
                                         bounds=(self._device_handle.ai_min_rate,
                                                 self._device_handle.ai_max_multi_chan_rate),
                                         increment=1,
                                         enforce_int=False), bin_width=None)

        for ch, src in self._analog_sources.items():
            self._channels[ch] = Channel(ch, unit=channel_units[ch],
                                     dtype=np.float64,
                                     streaming_mode=StreamingMode.CONTINUOUS, buffer_size=ScalarConstraint(default=1024**2,
                                                 bounds=(2, self._max_channel_samples_buffer),
                                                 increment=1,
                                                 enforce_int=True),
                                     sample_timing=SampleTiming.CONSTANT, sample_rate=ScalarConstraint(default=50.0,
                                         bounds=(self._device_handle.ai_min_rate,
                                                 self._device_handle.ai_max_multi_chan_rate),
                                         increment=1,
                                         enforce_int=False), bin_width=None)

        # Check external sample clock source
        if self._external_sample_clock_source is not None:
            new_name = self._extract_terminal(self._external_sample_clock_source)
            if new_name in self.__all_digital_terminals:
                self._external_sample_clock_source = new_name
            else:
                self.log.error(
                    f'No valid source terminal found for external_sample_clock_source '
                    f'"{self._external_sample_clock_source}". Falling back to internal sampling '
                    f'clock.'
                )
                self._external_sample_clock_source = None

        # Check external sample clock frequency
        if self._external_sample_clock_source is None:
            self._external_sample_clock_frequency = None
        elif self._external_sample_clock_frequency is None:
            self.log.error('External sample clock source supplied but no clock frequency. '
                           'Falling back to internal clock instead.')
            self._external_sample_clock_source = None
        elif not self._constraints.sample_rate.is_valid(self._external_sample_clock_frequency):
            self.log.error(
                f'External sample clock frequency requested '
                f'({self._external_sample_clock_frequency:.3e}Hz) is out of bounds. Please '
                f'choose a value between {self._constraints.sample_rate.minimum:.3e}Hz and '
                f'{self._constraints.sample_rate.maximum:.3e}Hz. Value will be clipped to the '
                f'closest boundary.'
            )
            self._external_sample_clock_frequency = self._constraints.sample_rate.clip(
                self._external_sample_clock_frequency
            )

        self._terminate_all_tasks()
        if self._external_sample_clock_frequency is None:
            sample_rate = self._constraints.sample_rate.default
        else:
            sample_rate = self._external_sample_clock_frequency
        self.configure(active_channels=self._constraints.channel_units,
                       streaming_mode=StreamingMode.CONTINUOUS,
                       channel_buffer_size=self._constraints.channel_buffer_size.default,
                       sample_rate=sample_rate)

    def on_deactivate(self):
        """ Shut down the NI card. """
        self._terminate_all_tasks()

    @property
    def name(self):
        """ Name of the hardware as string.

        @return str: The name of the hardware
        """
        return self._device_name

    @property
    def available_states(self):
        """ Names of the states as a dict of tuples.

        The keys contain the names for each of the switches. The values are tuples of strings
        representing the ordered names of available states for each switch.

        @return dict: Available states per switch in the form {"switch": ("state1", "state2")}
        """
        return self._switches.copy()

    def get_state(self, switch):
        """ Query state of single switch by name

        @param str switch: name of the switch to query the state for
        @return str: The current switch state
        """
        assert switch in self.available_states, f'Invalid switch name: "{switch}"'
        return self._states[switch]

    def set_state(self, switch, state):
        """ Query state of single switch by name

        @param str switch: name of the switch to change
        @param str state: name of the state to set
        """
        avail_states = self.available_states
        assert switch in avail_states, f'Invalid switch name: "{switch}"'
        assert state in avail_states[switch], f'Invalid state name "{state}" for switch "{switch}"'

        with nidaqmx.Task() as task:
            task.do_channels.add_do_chan('Dev1/port1/line0:3')
            task.start()
            while True:
                task.write([8, 0, 0, 0])

        self._states[switch] = state
