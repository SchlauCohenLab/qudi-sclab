# -*- coding: utf-8 -*-

__all__ = ['Newport8742Series']

import time
from collections import OrderedDict
from pylablib.devices import Newport
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

class Newport8742Series(ActuatorInterface):
    """
    Module for the picomotor Controller Kit Four-Axis (8742-4-KIT) sold by Newport.

    The controller takes commands of the form xxAAnn over a serial connection,
    where xx is the controller address and nn can be a value to be set or a question mark
    to get the value or it can be missing.

    Example config for copy-paste:

    newport_8742_series:
        module.Class: 'actuator.newport_8742_series.Newport8742Series'
        options:
            devices:
                device_1:
                    port: 'COM1'
                    axis_labels: ['x1', 'y1', 'z1', 'phi1']
                    axis_units: ['um', 'um', 'um', 'theta']
                device_2:
                    port: 'COM2'
                    axis_labels: ['x2', 'y2', 'z2', 'phi2']
                    axis_units: ['um', 'um', 'um', 'theta']

    """

    _devices = ConfigOption('devices', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()

        self._axes = dict()

    def on_activate(self):
        """
        Initialisation performed during activation of the module.
        """

        self._instr = []
        self._axes = dict()
        for port in self._devices:

            self._instr.append(Newport.Picomotor8742())


            for label, configs in self._axis.items():
                Axis(label, configs["unit"], (float(self.query(label, "SL")) * 1e-3,
                                              float(self.query(label, "SR")) * 1e-3),
                    step_range = (float(self.query(label, "SU")) * 1e-3, np.inf),
                    velocity_range=(0, float(self.query(label, "VA")) * 1e-3),
                    resolution_range = (0, 100000), frequency_range = (0, 1e3))

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        for label, configs in self._axis.items():
            self._devices[label].close()

    def query(self, axis_label, command):
        """

        :param axis_label:
        :param command:
        :return:
        """
        device = self._devices[axis_label]
        adress = self._axis[axis_label]['adress']
        return device.query("{}{}?".format(adress, command)).split(command)[1]

    def write(self, axis_label, command):
        """

        :param axis_label:
        :param command:
        :return:
        """
        device = self._devices[axis_label]
        adress = self._axis[axis_label]['adress']
        device.write("{}{}?".format(adress, command))

    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        constraints = OrderedDict()

        for label, configs in self._axis.items():
            axis = {
                'label': label,
                'ID': configs["adress"],
                'unit': configs["unit"],
                'ramp': None,
                'pos_min': float(self.query(label, "SL")) * 1e-3,
                'pos_max': float(self.query(label, "SR")) * 1e-3,
                'pos_step': float(self.query(label, "SU")) * 1e-3,
                'vel_min': float(self.query(label, "VA")) * 1e-3,
                'vel_max': float(self.query(label, "VA")) * 1e-3,
                'vel_step': None,

                'acc_min': float(self.query(label, "AC")) * 1e-3,
                'acc_max': float(self.query(label, "AC")) * 1e-3,
                'acc_step': None,
            }

            constraints[label] = axis

        return constraints

    def move_rel(self, axes_displacement):
        """ Moves stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        A smart idea would be to ask the position after the movement.

        @return int: error code (0:OK, -1:error)
        """
        pos_dict = {}
        for label, pos in param_dict.items():
            command = "PR{}".format(param_dict[label] * 1e3)
            self.write(label, command)
            pos_dict[label] = float(self.query(label, "TH")) * 1e-3

        return pos_dict

    def move_abs(self, axes_position):
        """ Moves stage to absolute position (absolute movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return int: error code (0:OK, -1:error)
        """
        pos_dict = {}
        for label, pos in param_dict.items():
            command = "PA{}".format(param_dict[label] * 1e3)
            self.write(label, command)
            pos_dict[label] = float(self.query(label, "TH")) * 1e-3

        return pos_dict

    def abort(self):
        """Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        for label in self._axis.keys():
            self.write(label, 'ST')
        return 0

    def get_pos(self, param_list=None):
        """ Gets current position of the rotation stage

        @param list param_list: List with axis name

        @return dict pos: Dictionary with axis name and pos in deg
        """
        if not param_list:
            param_list = [label for label in self._axis.keys()]
        pos_dict = {}
        for label in param_list:
            pos_dict[label] = float(self.query(label, "TP")) * 1e-3

        return pos_dict

    def get_status(self, param_list=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict status:
        """
        if not param_list:
            param_list = [label for label in self._axis.keys()]
        status_dict = {}
        for label in param_list:
            idx = self.query(label, "TS")[-2:]
            status = STATUS_dict[idx]
            self.log.debug("Newport CONEX Hardware status [axis {}] : {}".format(label, status))
            status_dict[label] = idx == '32' or idx == '33'

        return status_dict

    def calibrate(self, param_list=None):
        """ Calibrates the rotation actuator

        @param list param_list: Dictionary with axis name

        @return dict pos: Dictionary with axis name and pos in deg
        """
        if not param_list:
            param_list = [label for label in self._axis.keys()]
        pos_dict = {}
        for label in param_list:
            self.write(label, "OR")
            pos_dict[label] = float(self.query(label, "TH")) * 1e-3

        return pos_dict

    def get_velocity(self, param_list=None):
        """ Asks current value for velocity.

        @param list param_list: Dictionary with axis name

        @return dict velocity: Dictionary with axis name and velocity in deg/s
        """
        if not param_list:
            param_list = [label for label in self._axis.keys()]
        velocity_dict = {}
        for label in param_list:
            velocity_dict[label] = float(self.query(label, "VA")) * 1e-3

        return velocity_dict

    def set_velocity(self, param_dict):
        """ Write new value for velocity.

        @param dict param_dict: Dictionary with axis name and target velocity in deg/s

        @return dict velocity: Dictionary with axis name and target velocity in deg/s
        """
        velocity_dict = {}
        for label in param_dict.keys():
            velocity_dict[label] = float(self.query(label, "VA")) * 1e-3

        return velocity_dict

    def reset(self):
        """ Reset the controller.
            Afterwards, moving to the home position with calibrate() is necessary.
        """
        for label in self._axis.keys():
            self.write(label, 'RS')
        return 0
