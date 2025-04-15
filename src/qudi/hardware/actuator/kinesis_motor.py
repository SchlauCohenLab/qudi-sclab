# -*- coding: utf-8 -*-

"""
This module controls Kinesis brushed dc motor actuators made by Thorlabs.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import os
import sys
import numpy as np
import ctypes as ct
from qudi.core.configoption import ConfigOption
from interface.actuator_interface import ActuatorInterface, Axis
from qudi.util.mutex import Mutex
from pylablib.devices import Thorlabs

class KinesisMotor(ActuatorInterface):
    """ This class is implements communication with Thorlabs motors via Kinesis dll

    Example config for copy-paste:

    kinesis:
        module.Class: 'motor.kinesis_motor.KinesisMotor'
        options:
            dll_folder: 'C:\Program Files\Thorlabs\Kinesis'
            axes_cfg: 
                phi : 
                    serial_number: 000000123
                    polling_rate_ms: 100
                    scale: stage

    This hardware file have been develop for the TDC001/KDC101 rotation controller. It should work with other motors
    compatible with kinesis. Please be aware that Kinesis dll can be a little buggy sometimes.
    In particular conversion to real unit is sometimes broken. The following page helped me :
    https://github.com/MSLNZ/msl-equipment/issues/1

    """
    _axes_cfg = ConfigOption('axes_cfg', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()

    def on_activate(self):
        """
        Activate the module
        """

        self._axes = dict()
        self._devices = dict()

        for axis, cfg in self._axes_cfg.items():

            if serial_num not in dict(Thorlabs.list_kinesis_devices()).keys():
                self.log.error('Serial number {} not found in the list of connected devices'.format(cfg['serial_number']))
                continue

            if 'scale' not in cfg.keys():
                scale = 'stage'
            else:
                scale = cfg['scale']
                
            self._devices[axis] = Thorlabs.KinesisMotor(cfg['serial_number'], scale=scale)
            self.log.info('Found {} motor with serial number {}'.format(axis, cfg['serial_number']))


            units = self._devices[axis].get_scale_units()
            velocity_range = self._devices[axis].get_velocity_parameters()

            print(self._devices[axis].get_full_info())
            self._axes[axis] = Axis(axis, units, (0, int(360)), step_range=(0, np.inf),
                     resolution_range=(0, 100000), frequency_range=(0, 1e3), velocity_range=velocity_range)

    def on_deactivate(self):
        """ Disconnect from hardware on deactivation. """
        for axis, cfg in self._axes_cfg.items():
            self._devices[axis].close()

    def home(self, axes=None):
        """ Send a home instruction to a motor """
        if axes is None:
            axes = self._axes.keys()
        for axis in axes:
            self._devices[axis].home()

    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        return [axis for axis in self._axes.values()]

    def move_abs(self, positions):
        """ Move the scanning probe to an absolute position as fast as possible or with a defined
        velocity.

        Log error and return current target position if something fails or a scan is in progress.
        """
        for axis, pos in positions.items():
            if self._axes[axis].value_range[0] <= pos <= self._axes[axis].value_range[1]:
                self._devices[axis].move_to(pos)
            else:
                self.log.error('The input position of axis {} is outside the device range.'.format(axis))

    def move_rel(self, displacement):
        """ Move the scanning probe to an absolute position as fast as possible or with a defined
        velocity.

        Log error and return current target position if something fails or a scan is in progress.
        """
        current_pos = self.get_pos()
        for axis, dis in displacement.items():
            pos = current_pos[axis] + dis
            if self._axes[axis].value_range[0] <= pos <= self._axes[axis].value_range[1]:
                self._devices[axis].move_by(dis)
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
            pos[axis] = self._devices[axis].get_position()
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
            status[axis] = self._devices[axis].get_status()
        return status