# -*- coding: utf-8 -*-

"""
Interfuse of Ni Finite Sampling IO and NI AO HardwareFiles to make a confocal scanner.


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
from PySide2.QtGui import QGuiApplication

from qudi.interface.actuator_interface import ActuatorInterface, Axis
from qudi.interface.slow_counter_interface import SlowCounterInterface, CountingMode, SlowCounterConstraints
from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.util.mutex import RecursiveMutex, Mutex
from qudi.util.helpers import in_range
import ctypes as ct

__all__ = ['NanoDrive']

ERRORS = ['MCL_SUCCESS','MCL_GENERAL_ERROR','MCL_DEV_ERROR','MCL_DEV_NOT_ATTACHED','MCL_USAGE_ERROR','MCL_DEV_NOT_READY','MCL_ARGUMENT_ERROR','MCL_INVALID_AXIS','MCL_INVALID_HANDLE']

class NanoDrive(ActuatorInterface):
    """
    This interfuse combines modules of a National Instrument device to make up a scanning hardware.
    One module for software timed analog output (NIXSeriesAnalogOutput) to position e.g. a scanner to a specific
    position and a hardware timed module for in and output (NIXSeriesFiniteSamplingIO) to realize 1D/2D scans.

    Example config for copy-paste:

    nanodrive::
        module.Class: 'actuator.nanodrive.NanoDrive'
        options:
            dll_location: 'C:\\camera\\andor.dll' # path to library file
            axes_cfg:
                x_axis: 1
                y_axis: 2
                z_axis: 3

    """

    _dll_location = ConfigOption('dll_location', missing='error')
    _axes_cfg = ConfigOption('axes_cfg', missing='error')


    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()

        self._axes = dict()

    def on_activate(self):
        """
        Activate the module
        """
        try:
            self._dll = ct.windll.LoadLibrary(self._dll_location)
        except OSError:
            self.log.error('Error while loading the dll library, check the dll path.')
        # This module handle only one camera. DLL support up to 8 cameras.
        answer = self._dll.MCL_InitHandle()
        if answer == 0:
            self.log.error('Problem during device initialization')
            return
        if answer < 0:
            self.log.error('DEVICE ERROR : {}'.format(ERRORS[abs(answer)]))
            return
        self._device_handle = answer

        self._dll.MCL_SingleWriteN.argtypes = [ct.c_double, ct.c_int32, ct.c_int32]
        self._dll.MCL_SingleWriteN.restype = ct.c_int32

        self._dll.MCL_SingleReadN.argtypes = [ct.c_int32, ct.c_int32]
        self._dll.MCL_SingleReadN.restype = ct.c_double

        self._dll.MCL_GetCalibration.argtypes = [ct.c_int32, ct.c_int32]
        self._dll.MCL_GetCalibration.restype = ct.c_double

        self._axes = dict()
        for axis, port in self._axes_cfg.items():
            axis_range = self._dll.MCL_GetCalibration(port, self._device_handle)
            if axis_range == 0:
                self.log.error('An error occured when retrieving the axes range.')
            else:
                self._axes[axis] = Axis(axis, 'm', (0, int(axis_range)*1e-6), step_range=(0, np.inf),
                     resolution_range=(0, 100), frequency_range=(0, 1e3), velocity_range=(0, np.inf))


    def on_deactivate(self):
        """
        Deactivate the module
        """
        self._dll.MCL_ReleaseAllHandles()


    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        return [axis for axis in self._axes.values()]

    def home(self, axes=None):
        """ Hard reset of the hardware.
        """
        self.move_abs(self, {axis:0 for axis in self._axes.keys()})

    def move_abs(self, positions):
        """ Move the scanning probe to an absolute position as fast as possible or with a defined
        velocity.

        Log error and return current target position if something fails or a scan is in progress.
        """
        for axis, pos in positions.items():
            if self._axes[axis].value_range[0] <= pos <= self._axes[axis].value_range[1]:
                answer = self._dll.MCL_SingleWriteN(pos*1e6, self._axes_cfg[axis], self._device_handle)
                if answer < 0:
                    self.log.error('DEVICE ERROR : {}'.format(ERRORS[abs(answer)]))
                    return
            else:
                self.log.error('The input position of axis {} is outside the device range.'.format(axis))

    def move_rel(self, displacement):
        """ Move the scanning probe to an absolute position as fast as possible or with a defined
        velocity.

        Log error and return current target position if something fails or a scan is in progress.
        """
        current_pos = self.get_pos()
        for axis, dis in displacement.items():
            pos = current_pos + dis
            if self._axes[axis].value_range[0] <= pos <= self._axes[axis].value_range[1]:
                answer = self._dll.MCL_SingleWriteN(pos*1e6, self._axes_cfg[axis], self._device_handle)
                if answer < 0:
                    self.log.error('DEVICE ERROR : {}'.format(ERRORS[abs(answer)]))
                    return
            else:
                self.log.error('The input position of axis {} is outside the device range.'.format(axis))

    def get_pos(self, axes=None):
        """ Get a snapshot of the actual scanner position (i.e. from position feedback sensors).
        For the same target this value can fluctuate according to the scanners positioning accuracy.

        For scanning devices that do not have position feedback sensors, simply return the target
        position (see also: ScanningProbeInterface.get_target).

        @return dict: current position per axis.
        """
        pos = {}
        if axes is None:
            axes = self._axes.keys()
        for axis in axes:
            answer = self._dll.MCL_SingleReadN(self._axes_cfg[axis], self._device_handle)
            if answer < 0:
                self.log.error('DEVICE ERROR : {}'.format(ERRORS[abs(answer)]))
            else:
                pos[axis] = answer*1e-6
        return pos

    def abort(self, axes=None):
        """

        @return:
        """
        pass

    def get_status(self, axes=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        """
        status = {}
        if axes is None:
            axes = self._axes.keys()
        for axis in axes:
            status[axis] = 'OK'
        return status