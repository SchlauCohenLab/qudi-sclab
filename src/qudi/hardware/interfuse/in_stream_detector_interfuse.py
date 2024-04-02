# -*- coding: utf-8 -*-

"""
Combine two hardware switches into one.

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
import numpy as np
import time

from PySide2 import QtCore

from qudi.interface.scanner_interface import ScannerInterface, ScanData, ScanConstraints
from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.util.mutex import RecursiveMutex

from qudi.interface.detector_interface import DetectorInterface, Channel

class InStreamDetectorInterfuse(DetectorInterface):
    """ Methods to control slow (mechanical) laser switching devices.
    This interfuse in particular combines two switches into one.

    Example config for copy-paste:

    in_stream_detector:
        module.Class: 'interfuse.in_stream_detector_interfuse.InStreamDetectorInterfuse'
        connect:
            streamer: 'streamer'
        options:
            active_channels:
                - donor:
                    - port: PFI0
                    - unit: count/s
                    - streaming_mode: FINITE
                    - streaming_mode: FINITE
                - acceptor: PFI12

    """

    # connectors for the switches to be combined
    _streamer = Connector(name='streamer', interface='DataInStreamInterface')

    _active_channels = ConfigOption(name='active_channels', default=None, missing='warning')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.mutex = RecursiveMutex()

        self._channels = dict()

    def on_activate(self):
        """ Activate the module and fill status variables.
        """
        
        if self._active_channels is None:
            self._active_channels = {ch:ch for ch in self._streamer.active_channels}
        for name, channel in self._active_channels.items():
            self._channels[name] = Channel(name=name, unit='', dtype=np.float64, streaming_mode=StreamingMode.FINITE, buffer_size=1,
                 sample_timing=SampleTiming.CONSTANT, sample_rate=None, bin_width=None):

    def on_deactivate(self):
        """ Deactivate the module and clean up.
        """

    def get_constraints(self):
        """ Read-only property returning the constraints on the settings for this data streamer. """
        pass

    def configure_channels(self, channels_settings):
        """ Configure a data stream. See read-only properties for information on each parameter. """
        pass

    def start_measure(self):
        """ Start the fast counter. """
        pass

    def stop_measure(self):
        """ Stop the fast counter. """
        pass

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