# -*- coding: utf-8 -*-

"""
Hardware module to control a Mad City Labs MicroDrive motorized/encoder stage as a qudi
actuator. This is a thin ActuatorInterface adapter around the vendored wrapper in
mcl_microdrive_lib.py (from https://github.com/miseitz/MCLMicroDrive, MIT licensed), which does
the actual ctypes/DLL work - see that file for the underlying driver and its license notice.

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

-----------------------------------------------------------------------------------------------
HARDWARE NOTE
-----------------------------------------------------------------------------------------------
Unlike the Nano-Drive (a closed-loop piezo stage addressed with absolute-position writes via
MCL_SingleWriteN/MCL_SingleReadN), the Mad City Labs MicroDrive is a motorized stage with
encoders whose vendored driver (mcl_microdrive_lib.MicroDrive) is RELATIVE-move based and
built around fixed MCL axis numbers 1, 2 and 3 only (its getPosition()/moveAxis()/etc. always
work in terms of exactly axes 1-3; a 4th physical axis, if your stage has one, is not supported
by the vendored wrapper). Movements and velocities in the vendored driver are in mm and mm/s;
this adapter converts to/from qudi's meters/(m/s) convention at the boundary.

The vendored driver also reports status and errors via print() to stdout rather than through
qudi's logger - that output will show up in whatever console qudi was launched from, not in the
qudi log window, since this module does not modify that behavior (see mcl_microdrive_lib.py for
why: it's kept close to the upstream wrapper on purpose).

Example config for copy-paste:

microdrive:
    module.Class: 'actuator.microdrive.MicroDrive'
    options:
        dll_location: 'C:\\Program Files\\Mad City Labs\\MicroDrive\\MicroDrive'  # path to library file, no .dll extension needed
        axes_cfg:
            x_axis: 1
            y_axis: 2
            z_axis: 3
        velocity: 3.0  # optional, move velocity in mm/s (vendored driver's native unit); clipped to the device's min/max

"""

import numpy as np

from qudi.interface.actuator_interface import ActuatorInterface, Axis
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.hardware.actuator.mcl_microdrive_lib import MicroDrive as _MCLMicroDriveDriver

__all__ = ['MicroDrive']


class MicroDrive(ActuatorInterface):
    """
    Hardware module for a Mad City Labs MicroDrive motorized/encoder stage, delegating to the
    vendored mcl_microdrive_lib.MicroDrive driver for all DLL interaction.

    Example config for copy-paste:

    microdrive:
        module.Class: 'actuator.microdrive.MicroDrive'
        options:
            dll_location: 'C:\\Program Files\\Mad City Labs\\MicroDrive\\MicroDrive'
            axes_cfg:
                x_axis: 1
                y_axis: 2
                z_axis: 3
            velocity: 3.0  # optional, mm/s

    """

    _dll_location = ConfigOption('dll_location', missing='error')
    _axes_cfg = ConfigOption('axes_cfg', missing='error')
    _configured_velocity = ConfigOption('velocity', default=None, missing='nothing')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()
        self._axes = dict()
        self._driver = None
        self._velocity = 3.0

    def on_activate(self):
        """
        Activate the module
        """
        for axis, port in self._axes_cfg.items():
            if port not in (1, 2, 3):
                raise ValueError('axes_cfg entry "{}" -> {} is invalid: the vendored MicroDrive '
                                 'driver only supports MCL axis numbers 1, 2 or 3.'
                                 ''.format(axis, port))

        self._driver = _MCLMicroDriveDriver(mcl_lib=self._dll_location)
        if self._driver.handle <= 0:
            self.log.error('Problem during device initialization. Is the MicroDrive connected '
                           'and turned on?')
            self._driver = None
            return

        if self._configured_velocity is None:
            self._velocity = self._driver.velocityMax
        else:
            self._velocity = min(max(self._configured_velocity, self._driver.velocityMin),
                                 self._driver.velocityMax)

        half_range_m = (self._driver.totalScanRange * 1e-3) / 2
        velocity_range_m_s = (self._driver.velocityMin * 1e-3, self._driver.velocityMax * 1e-3)
        self._axes = dict()
        for axis in self._axes_cfg:
            self._axes[axis] = Axis(axis, 'm', (-half_range_m, half_range_m),
                                    step_range=(0, np.inf), resolution_range=(0, 100000),
                                    frequency_range=(0, 1e3), velocity_range=velocity_range_m_s)

    def on_deactivate(self):
        """
        Deactivate the module
        """
        if self._driver is not None:
            self._driver.closeConnection()

    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        return [axis for axis in self._axes.values()]

    def home(self, axes=None):
        """ Moves the specified (or all) axes to their zero position.
        """
        if axes is None or set(axes) >= set(self._axes):
            self._driver.Home()
        else:
            for axis in axes:
                self._driver.moveControlledAxis(self._axes_cfg[axis], 0.0, self._velocity)

    def move_abs(self, positions):
        """ Move the stage to an absolute position per axis (in meters).

        Log error and skip the axis if the target is out of range.
        """
        for axis, pos in positions.items():
            if self._axes[axis].value_range[0] <= pos <= self._axes[axis].value_range[1]:
                self._driver.moveControlledAxis(self._axes_cfg[axis], pos * 1e3, self._velocity)
            else:
                self.log.error('The input position of axis {} is outside the device range.'
                               ''.format(axis))

    def move_rel(self, displacement):
        """ Move the stage by a relative displacement per axis (in meters).

        Log error and skip the axis if the target is out of range.
        """
        current_pos = self.get_pos(displacement.keys())
        for axis, dis in displacement.items():
            if axis not in current_pos:
                continue
            pos = current_pos[axis] + dis
            if self._axes[axis].value_range[0] <= pos <= self._axes[axis].value_range[1]:
                self._driver.moveRelativeAxis(self._axes_cfg[axis], dis * 1e3, self._velocity)
            else:
                self.log.error('The input position of axis {} is outside the device range.'
                               ''.format(axis))

    def get_pos(self, axes=None):
        """ Get a snapshot of the actual stage position from the encoders.

        @return dict: current position per axis, in meters.
        """
        if axes is None:
            axes = self._axes.keys()
        axes = list(axes)
        if not axes:
            return {}

        position_mm = self._driver.getPosition()  # always [axis1, axis2, axis3] in mm
        pos = {}
        for axis in axes:
            port = self._axes_cfg[axis]
            pos[axis] = position_mm[port - 1] * 1e-3
        return pos

    def abort(self, axes=None):
        """ Stops all motion immediately.

        @return int: error code (0:OK, -1:error)
        """
        self._driver.stopMoving()
        return 0

    def get_status(self, axes=None):
        """ Get the limit-switch status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and a status string as item.
        """
        if axes is None:
            axes = self._axes.keys()

        limits = self._driver.getStatus()  # list of [axis, direction, description]
        active_limits = {entry[0]: entry[2] for entry in limits if entry[0] != 0}

        status = {}
        for axis in axes:
            port = self._axes_cfg[axis]
            status[axis] = active_limits.get(port, 'OK')
        return status
