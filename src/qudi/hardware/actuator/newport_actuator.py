# -*- coding: utf-8 -*-

__all__ = ['NewportActuator']

import time
from collections import OrderedDict
import pyvisa
from qudi.interface.actuator_interface import ActuatorInterface, Axis, AxisStatus
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
import numpy as np

STATUS_dict = {
    '0A': "NOT REFERENCED from RESET.",
    '0B': "NOT REFERENCED from HOMING.",
    '0C': "NOT REFERENCED from CONFIGURATION.",
    '0D': "NOT REFERENCED from DISABLE.",
    '0E': "NOT REFERENCED from READY.",
    '0F': "NOT REFERENCED from MOVING.",
    '10': "NOT REFERENCED - NO PARAMETERS IN MEMORY.",
    '14': "CONFIGURATION",
    '1E': "HOMING.",
    '28': "MOVING.",
    '32': "READY from HOMING.",
    '33': "READY from MOVING.",
    '34': "READY from DISABLE.",
    '36': "READY T from READY.",
    '37': "READY T from TRACKING.",
    '38': "READY T from DISABLE T.",
    '3C': "DISABLE from READY.",
    '3D': "DISABLE from MOVING.",
    '3E': "DISABLE from TRACKING.",
    '3F': "DISABLE from READY T.",
    '46': "TRACKING from READY T.",
    '47': "TRACKING from TRACKING",
}

class NewportActuator(ActuatorInterface):
    """
    Module for the picomotor Controller Kit Four-Axis (8742-4-KIT) sold by Newport.

    The controller takes commands of the form xxAAnn over a serial connection,
    where xx is the controller address and nn can be a value to be set or a question mark
    to get the value or it can be missing.

    Example config for copy-paste:

     newport_8743_series:
        module.Class: 'actuator.newport_874x_series.Newport874xSeries'
        options:
            baud_rate: 921600
            port: 'USB0::0x104D::0x4000::12345678::RAW'
            axes:
                x1:
                    axis: 0
                    step: 30e-9
                    unit: m
                x2:
                    axis: 1
                    unit: m
                    step: 30e-9

    """

    _baud_rate = ConfigOption('baud_rate', default=921600)
    _port = ConfigOption('port', missing='error')
    _axes_cfg = ConfigOption('axes', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()

        self._axes = dict()

    def on_activate(self):
        """
        Initialisation performed during activation of the module.
        """
        self._rm = pyvisa.ResourceManager()

        self._device = self._rm.open_resource(self._port)
        self._device.baud_rate = self._baud_rate
        self._device.read_termination = "\r\n"
        self._device.write_termination = "\r"

        self._axes = {}
        for axis, cfg in self._axes_cfg.items():

            #self.write(axis, 'OR')

            self._axes[axis] = Axis(axis, cfg["unit"], (-2147483648 * cfg["step"], 2147483647 * cfg["step"]),
                step_range = (-2147483648, +2147483647),
                velocity_range=(0, float(self.query(axis, "VA")) * cfg["step"]),
                resolution_range = (1, 100000),
                frequency_range = (0, 1e3))

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self._device.close()

    def query(self, axis_label, command):
        """

        :param axis_label:
        :param command:
        :return:
        """
        adress = self._axes_cfg[axis_label]['axis']
        return self._device.query("0{}{}?".format(adress, command))#.split(command)[1]

    def write(self, axis_label, command):
        """
        :param axis_label:
        :param command:
        :return:
        """
        adress = self._axes_cfg[axis_label]['axis']
        self._device.write("0{}{}?".format(adress, command))

    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        return [axis for axis in self._axes.values()]

    def move_rel(self, axes_displacement):
        """ Moves stage in given direction (relative movement)

        @param dict axes_displacement: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        A smart idea would be to ask the position after the movement.

        @return int: error code (0:OK, -1:error)
        """
        pos_dict = {}
        for axis, dis in axes_displacement.items():
            command = "PR{}".format(int(dis/self._axes_cfg[axis]['step']))
            self.write(axis, command)
            pos_dict[axis] = float(self.query(axis, "TP")) * self._axes_cfg[axis]['step']

        return pos_dict

    def move_abs(self, axes_position):
        """ Moves stage to absolute position (absolute movement)

        @param dict axes_position: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return int: error code (0:OK, -1:error)
        """
        pos_dict = {}
        for axis, dis in axes_position.items():
            command = "PA{}".format(int(dis/self._axes_cfg[axis]['step']))
            self.write(axis, command)
            pos_dict[axis] = float(self.query(axis, "TP")) * self._axes_cfg[axis]['step']

        return pos_dict

    def abort(self):
        """Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        for label in self._axis.keys():
            self.write(label, 'ST')
        return 0

    def get_pos(self, axes=None):
        """ Gets current position of the rotation stage

        @param list param_list: List with axis name

        @return dict pos: Dictionary with axis name and pos in deg
        """
        pos = {}
        if axes is None:
            axes = self._axes.keys()
        for axis in axes:
            pos[axis] = float(self.query(axis, "TP")) * self._axes_cfg[axis]['step']

        return pos

    def get_status(self, axes=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict status:
        """
        status = {}
        if axes is None:
            axes = self._axes.keys()
        for axis in axes:
            idx = self.query(axis, "TS")[-2:]
            self.log.debug("Newport CONEX Hardware status [axis {}] : {}".format(axis, STATUS_dict[idx]))
            status[axis] = idx == '32' or idx == '33'

        return status

    def home(self, axes=None):
        """ Calibrates the rotation actuator

        @param list param_list: Dictionary with axis name

        @return dict pos: Dictionary with axis name and pos in deg
        """
        if axes is None:
            axes = self._axes.keys()
        for axis in axes:
            self.write(axis, "OR")

    def reset(self):
        """ Reset the controller.
            Afterwards, moving to the home position with calibrate() is necessary.
        """
        for label in self._axis.keys():
            self.write(label, 'RS')
        return 0