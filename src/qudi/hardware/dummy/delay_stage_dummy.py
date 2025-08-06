# -*- coding: utf-8 -*-

from collections import OrderedDict
import pyvisa
from qudi.interface.actuator_interface import ActuatorInterface, Axis, AxisStatus
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
import numpy as np


class ActuatorDummy(ActuatorInterface):
    """
    Module for the picomotor Controller Kit Four-Axis (8742-4-KIT) sold by Newport.

    The controller takes commands of the form xxAAnn over a serial connection,
    where xx is the controller address and nn can be a value to be set or a question mark
    to get the value or it can be missing.

    Example config for copy-paste:

     newport_8743_series:
        module.Class: 'dummy.delay_stage_dummy.ActuatorDummy'
            axes:
                x1:
                    axis: 1
                    unit: m
                    min: 2.457e-2
                    max: 7.543e-2
    """
    _axes_cfg = ConfigOption('axes', missing='error')
    _pos = 0

    def on_activate(self):
        """
        Initialisation performed during activation of the module.
        """
        self._axes = {}
        for axis, cfg in self._axes_cfg.items():
            self._axes[axis] = Axis(axis, cfg["unit"], (cfg["min"], cfg["max"]),
                step_range=(int(cfg["min"]/1e-3), int(cfg["max"]/1e-3)),
                velocity_range=(0, 100),
                resolution_range=(1, 100000),
                frequency_range=(0, 1e3))

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        pass

    def query(self, axis_label, command):
        """

        :param axis_label:
        :param command:
        :return:
        """
        pass

    def write(self, axis_label, command):
        """
        :param axis_label:
        :param command:
        :return:
        """
        pass

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
        for axis, dis in axes_displacement.items():
            self._pos += dis/1e-3
        return self._pos

    def move_abs(self, axes_position):
        """ Moves stage to absolute position (absolute movement)

        @param dict axes_position: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                {'axis_label': <the-abs-pos-value>}.
                                'axis_label' must correspond to a label given
                                to one of the axis.

        @return int: error code (0:OK, -1:error)
        """

        for axis, dis in axes_position.items():
            self._pos = dis/1e-3

        return self._pos


    def abort(self):
        """Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
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
            pos[axis] = self._pos * 1e-3

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
        pass

    def home(self, axes=None):
        """ Calibrates the rotation actuator

        @param list param_list: Dictionary with axis name

        @return dict pos: Dictionary with axis name and pos in deg
        """
        pass

    def reset(self):
        """ Reset the controller.
            Afterwards, moving to the home position with calibrate() is necessary.
        """
        pass
    
    def get_speed(self, axes=None):
        """ Get the vecolity of the stage
        @param list param_list: optional, if a specific speed of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                speed is asked.
        @return dict: with the axis label as key and the speed value as item.

        """
        return 1

    def set_speed(self, axes_velocity):
        """ Set the velocty of the stage
        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                {'axis_label': <the-abs-vel-value>}.
                                'axis_label' must correspond to a label given
                                to one of the axis, e.g x1.
        @return dict: int: error code (0:OK, -1:error)
        """
        pass