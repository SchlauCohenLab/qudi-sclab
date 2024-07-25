# -*- coding: utf-8 -*-

__all__ = ['NewportESP300']

import time
from collections import OrderedDict
import pyvisa
from qudi.interface.actuator_interface import ActuatorInterface, Axis, AxisStatus
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
import numpy as np

STATUS_dict = {
        '1': 'PCI COMMUNICATION TIME-OUT',
        '4': 'EMERGENCY SOP ACTIVATED',
        '6': 'COMMAND DOES NOT EXIST',
        '7': 'PARAMETER OUT OF RANGE',
        '8': 'CABLE INTERLOCK ERROR',
        '9': 'AXIS NUMBER OUT OF RANGE',
        '13': 'GROUP NUMBER MISSING',
        '14': 'GROUP NUMBER OUT OF RANGE',
        '15': 'GROUP NUMBER NOT ASSIGNED',
        '17': 'GROUP AXIS OUT OF RANGE',
        '18': 'GROUP AXIS ALREADY ASSIGNED',
        '19': 'GROUP AXIS DUPLICATED',
        '16': 'GROUP NUMBER ALREADY ASSIGNED',
        '20': 'DATA ACQUISITION IS BUSY',
        '21': 'DATA ACQUISITION SETUP ERROR',
        '23': 'SERVO CYCLE TICK FAILURE',
        '25': 'DOWNLOAD IN PROGRESS',
        '26': 'STORED PROGRAM NOT STARTED',
        '27': 'COMMAND NOT ALLOWED',
        '29': 'GROUP PARAMETER MISSING',
        '30': 'GROUP PARAMETER OUT OF RANGE',
        '31': 'GROUP MAXIMUM VELOCITY EXCEEDED',
        '32': 'GROUP MAXIMUM ACCELERATION EXCEEDED',
        '22': 'DATA ACQUISITION NOT ENABLED',
        '28': 'STORED PROGRAM FLASH AREA FULL',
        '33': 'GROUP MAXIMUM DECELERATION EXCEEDED',
        '35': 'PROGRAM NOT FOUND',
        '37': 'AXIS NUMBER MISSING',
        '38': 'COMMAND PARAMETER MISSING',
        '34': 'GROUP MOVE NOT ALLOWED DURING MOTION',
        '39': 'PROGRAM LABEL NOT FOUND',
        '40': 'LAST COMMAND CANNOT BE REPEATED',
        '41': 'MAX NUMBER OF LABELS PER PROGRAM EXCEEDED',
        '00': 'MOTOR TYPE NOT DEFINED',
        '01': 'PARAMETER OUT OF RANGE',
        '02': 'AMPLIFIER FAULT DETECTED',
        '03': 'FOLLOWING ERROR THRESHOLD EXCEEDED',
        '04': 'POSITIVE HARDWARE LIMIT DETECTED',
        '05': 'NEGATIVE HARDWARE LIMIT DETECTED',
        '06': 'POSITIVE SOFTWARE LIMIT DETECTED',
        '07': 'NEGATIVE SOFTWARE LIMIT DETECTED',
        '08': 'MOTOR / STAGE NOT CONNECTED',
        '09': 'FEEDBACK SIGNAL FAULT DETECTED',
        '10': 'MAXIMUM VELOCITY EXCEEDED',
        '11': 'MAXIMUM ACCELERATION EXCEEDED',
        '12': 'Reserved for future use',
    }

class NewportESP300(ActuatorInterface):
    """
    Module for the picomotor Controller Kit Four-Axis (8742-4-KIT) sold by Newport.

    The controller takes commands of the form xxAAnn over a serial connection,
    where xx is the controller address and nn can be a value to be set or a question mark
    to get the value or it can be missing.

    Example config for copy-paste:

     newport_8743_series:
        module.Class: 'actuator.newport_874x_series.Newport874xSeries'
        options:
            port: 'COM1'
            axes:
                x1:
                    axis: 0
                    unit: m
                    min: 2.457e-2
                    max: 7.543e-2
                x2:
                    axis: 1
                    unit: m
                    min: 2.457e-2
                    max: 7.543e-2

    """

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
        self._device.baud_rate = 19200
        self._device.read_termination = "\r\n"
        self._device.write_termination = "\r"

        self._axes = {}
        for axis, cfg in self._axes_cfg.items():

            #self.write(axis, 'OR')

            self._axes[axis] = Axis(axis, cfg["unit"], (cfg["min"], cfg["max"]),
                step_range=(int(cfg["min"]/1e-3), int(cfg["max"]/1e-3)),
                velocity_range=(0, float(self.query(axis, "VA"))*1e-3),
                resolution_range=(1, 100000),
                frequency_range=(0, 1e3))

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
            command = "PR{}".format(round(dis/1e-3, 3))
            self.write(axis, command)
            pos_dict[axis] = float(self.query(axis, "TP")) * 1e-3

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
            command = "PA{}".format(round(dis/1e-3, 3))
            self.write(axis, command)
            pos_dict[axis] = float(self.query(axis, "TP")) * 1e-3

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
            pos[axis] = float(self.query(axis, "TP")) * 1e-3

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
