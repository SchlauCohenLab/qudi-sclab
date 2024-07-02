# -*- coding: utf-8 -*-

__all__ = ['NewportMotor']

import time
from collections import OrderedDict
import visa
from qudi.interface.actuator_interface import ActuatorInterface
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex

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

class NewportMotor(MotorInterface):
    """
    Module for the CONEX controller for Agilis stages sold by Newport.

    The controller takes commands of the form xxAAnn over a serial connection,
    where xx is the controller address and nn can be a value to be set or a question mark
    to get the value or it can be missing.


    Example config for copy-paste:

    newport_conex:
        module.Class: 'actuator.motor_newport_conex.MotorNewportConex'
        options:
            axis:
                x1:
                    port: 'COM5'
                    adress: '01'
                    unit: 'm'
                x2:
                    port: 'COM7'
                    adress: '01'
                    unit: 'm'
                y1:
                    port: 'COM8'
                    adress: '01'
                    unit: 'm'
                y2:
                    port: 'COM9'
                    adress: '01'
                    unit: 'm'

    """

    _axis = ConfigOption('axis', missing='error')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        self._rm = visa.ResourceManager()

        self._devices = {}
        for label, configs in self._axis.items():
            device = self._rm.open_resource(configs["port"])
            device.baud_rate = 19200
            device.read_termination = "\r\n"

            self._devices[label] = device

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
        """ Retrieve the hardware constrains from the actuator device.

        @return dict: dict with constraints for the sequence generation and GUI

        Provides all the constraints for the xyz stage  and rot stage (like total
        movement, velocity, ...)
        Each constraint is a tuple of the form
            (min_value, max_value, stepsize)
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

    def move_rel(self, param_dict):
        """Moves stage by a given angle (relative movement)

        @param dict param_dict: Dictionary with axis name and relative movement in units

        @return dict: Dictionary with axis name and final position in units
        """
        pos_dict = {}
        for label, pos in param_dict.items():
            command = "PR{}".format(param_dict[label] * 1e3)
            self.write(label, command)
            pos_dict[label] = float(self.query(label, "TP")) * 1e-3

        return pos_dict

    def move_abs(self, param_dict):
        """Moves stage to an absolute angle (absolute movement)

        @param dict param_dict: Dictionary with axis name and target position in deg

        @return dict velocity: Dictionary with axis name and final position in deg
        """
        pos_dict = {}
        for label, pos in param_dict.items():
            command = "PA{}".format(param_dict[label] * 1e3)
            self.write(label, command)
            pos_dict[label] = float(self.query(label, "TP")) * 1e-3

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
            pos_dict[label] = float(self.query(label, "TP")) * 1e-3

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
