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

class KinesisMotor(ActuatorInterface):
    """ This class is implements communication with Thorlabs motors via Kinesis dll

    Example config for copy-paste:

    kinesis:
        module.Class: 'motor.kinesis_motor.KinesisMotor'
        options:
            dll_folder: 'C:\Program Files\Thorlabs\Kinesis'
            axes_cfg: 
                phi : 000000123
                teta : 000000123

    This hardware file have been develop for the TDC001/KDC101 rotation controller. It should work with other motors
    compatible with kinesis. Please be aware that Kinesis dll can be a little buggy sometimes.
    In particular conversion to real unit is sometimes broken. The following page helped me :
    https://github.com/MSLNZ/msl-equipment/issues/1

    """
    _dll_folder = ConfigOption('dll_folder', default=r"C:\Program Files\Thorlabs\Kinesis")
    _dll_file = ConfigOption('dll_file', default="Thorlabs.MotionControl.KCube.DCServo.dll")
    _axes_cfg = ConfigOption('axes_cfg', missing='error')
    _polling_rate_ms = ConfigOption('polling_rate_ms', default=200)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()

        self._axes = dict()

    def on_activate(self):
        """
        Activate the module
        """

        if sys.version_info < (3, 8):
            os.chdir(self._dll_folder)
        else:
            os.add_dll_directory(r"C:\Program Files\Thorlabs\Kinesis")
            
        self._dll = ct.cdll.LoadLibrary(self._dll_file)
        #except OSError:
        #    self.log.error('Error while loading the dll library, check the dll path.')
        
        self._dll.TLI_BuildDeviceList()

        self._axes = dict()

        for axis, sn in self._axes_cfg.items():
            serial_number = ct.c_char_p(str(sn).encode('utf-8'))
            self._dll.BMC_Open(serial_number)
            self._dll.BMC_LoadSettings(serial_number)
            self._dll.BMC_StartPolling(serial_number, ct.c_int(self._polling_rate_ms))
            #max_pos = self._dll.BMC_GetStageAxisInfo_MaxPos(serial_number)
            #print(max_pos)
            
            self._axes[axis] = Axis(axis, 'm', (0, int(360)), step_range=(0, np.inf),
                     resolution_range=(0, 100000), frequency_range=(0, 1e3), velocity_range=(0, np.inf))


    def on_deactivate(self):
        """ Disconnect from hardware on deactivation. """
        for axis, sn in self._axes_cfg.items():
            self._dll.BMC_ClearMessageQueue(sn)
            self._dll.BMC_StopPolling(sn)
            self._dll.BMC_Close(sn)

    def get_position(self, name):
        """ Get the position in real work unit of the motor """
        serial_number = self._axes_cfg[name]
        position = self._dll.BMC_GetPosition(serial_number)
        real_unit = ct.c_double()
        self._dll.BMC_GetRealValueFromDeviceUnit(serial_number, position, ct.byref(real_unit), 0)
        return real_unit.value

    def set_position(self, name, value):
        """ Set the position in real work unit of an axis """
        serial_number = self._axes_cfg[name]
        device_unit = ct.c_int()
        self._dll.BMC_GetDeviceUnitFromRealValue(serial_number, ct.c_double(value), ct.byref(device_unit), 0)
        self._dll.BMC_MoveToPosition(serial_number, device_unit)

    def home(self, axes=None):
        """ Send a home instruction to a motor """
        if axes is None:
            axes = self._axes.keys()
        for axis in axes:
            serial_number = self._axes_cfg[axis]
            self._dll.BMC_Home(serial_number)

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
                self.set_position(axis, pos)
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
                self.set_position(axis, pos)
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
            pos[axis] = self.get_position(axis)
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