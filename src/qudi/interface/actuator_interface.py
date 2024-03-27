# -*- coding: utf-8 -*-

__all__ = ['ActuatorInterface']

from abc import abstractmethod
from PySide2 import QtCore

from qudi.core.module import Base
from enum import Enum
import datetime
import numpy as np

class AxisStatus(Enum):
    OK = 0
    ERROR = -1

class Axis:
    """
    """

    def __init__(self, name, unit='', value_range=(-np.inf, np.inf), step_range=(0, np.inf),
                 resolution_range=(1, np.inf), frequency_range=(0, np.inf), velocity_range=(0, np.inf)):

        if not isinstance(name, str):
            raise TypeError('Parameter "name" must be of type str.')
        if name == '':
            raise ValueError('Parameter "name" must be non-empty str.')
        if not isinstance(unit, str):
            raise TypeError('Parameter "unit" must be of type str.')
        if not (len(value_range) == len(step_range) == len(resolution_range) == len(
                frequency_range) == 2):
            raise ValueError('Range parameters must be iterables of length 2')

        self._name = name
        self._unit = unit
        self._resolution_range = (int(min(resolution_range)), int(max(resolution_range))) #TODO np.inf cannot be casted as an int
        self._step_range = (float(min(step_range)), float(max(step_range)))
        self._value_range = (float(min(value_range)), float(max(value_range)))
        self._frequency_range = (float(min(frequency_range)), float(max(frequency_range)))
        self._velocity_range = (float(min(velocity_range)), float(max(velocity_range)))

    def __eq__(self, other):
        if not isinstance(other, Axis):
            raise NotImplemented
        attrs = ('_name',
                 '_unit',
                 '_resolution_range',
                 '_step_range',
                 '_value_range',
                 '_frequency_range',
                 '_velocity',
                 )
        return all(getattr(self, a) == getattr(other, a) for a in attrs)

    @property
    def name(self):
        return self._name

    @property
    def unit(self):
        return self._unit

    @property
    def resolution_range(self):
        return self._resolution_range

    @property
    def min_resolution(self):
        return self._resolution_range[0]

    @property
    def max_resolution(self):
        return self._resolution_range[1]

    @property
    def step_range(self):
        return self._step_range

    @property
    def min_step(self):
        return self._step_range[0]

    @property
    def max_step(self):
        return self._step_range[1]

    @property
    def value_range(self):
        return self._value_range

    @property
    def min_value(self):
        return self._value_range[0]

    @property
    def max_value(self):
        return self._value_range[1]

    @property
    def frequency_range(self):
        return self._frequency_range

    @property
    def min_frequency(self):
        return self._frequency_range[0]

    @property
    def max_frequency(self):
        return self._frequency_range[1]

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, velocity):
        if not isinstance(velocity, float):
            raise TypeError('Parameter "velocity" must be of type float.')
        self._velocity = velocity

    def clip_value(self, value):
        if value < self.min_value:
            return self.min_value
        elif value > self.max_value:
            return self.max_value
        return value

    def clip_resolution(self, res):
        if res < self.min_resolution:
            return self.min_resolution
        elif res > self.max_resolution:
            return self.max_resolution
        return res

    def clip_frequency(self, freq):
        if freq < self.min_frequency:
            return self.min_frequency
        elif freq > self.max_frequency:
            return self.max_frequency
        return freq

    def to_dict(self):
        dict_repr = {'name': self._name,
                     'unit': self._unit,
                     'value_range': self._value_range,
                     'step_range': self._step_range,
                     'resolution_range': self._resolution_range,
                     'frequency_range': self._frequency_range}
        return dict_repr

    @classmethod
    def from_dict(cls, dict_repr):
        return Axis(**dict_repr)

class ActuatorInterface(Base):
    """ This is the Interface class to control an actuator device. The actual hardware implementation might have a
        different amount of axis. Implement each single axis as 'private'
        methods for the hardware class, which get called by the general method.
    """

    @abstractmethod
    def get_constraints(self):
        """ Retrieve the hardware constrains from the actuator device.

        @return dict: dict with constraints for the magnet hardware. These
                      constraints will be passed via the logic to the GUI so
                      that proper display elements with boundary conditions
                      could be made.

        Provides all the constraints for each axis of a motorized stage
        (like total travel distance, velocity, ...)
        Each axis has its own dictionary, where the label is used as the
        identifier throughout the whole module. The dictionaries for each axis
        are again grouped together in a constraints dictionary in the form

            {'<label_axis0>': axis0 }

        where axis0 is again a dict with the possible values defined below. The
        possible keys in the constraint are defined here in the interface file.
        If the hardware does not support the values for the constraints, then
        insert just None. If you are not sure about the meaning, look in other
        hardware files to get an impression.

        Example of how a return dict with constraints might look like:
        ==============================================================

        constraints = {}

        axis0 = {}
        axis0['label'] = 'x'    # it is very crucial that this label coincides
                                # with the label set in the config.
        axis0['unit'] = 'm'     # the SI units, only possible m or degree
        axis0['ramp'] = ['Sinus','Linear'], # a possible list of ramps
        axis0['pos_min'] = 0,
        axis0['pos_max'] = 100,  # that is basically the traveling range
        axis0['pos_step'] = 100,
        axis0['vel_min'] = 0,
        axis0['vel_max'] = 100,
        axis0['vel_step'] = 0.01,
        axis0['acc_min'] = 0.1
        axis0['acc_max'] = 0.0
        axis0['acc_step'] = 0.0

        axis1 = {}
        axis1['label'] = 'phi'   that axis label should be obtained from config
        axis1['unit'] = 'degree'        # the SI units
        axis1['ramp'] = ['Sinus','Trapez'], # a possible list of ramps
        axis1['pos_min'] = 0,
        axis1['pos_max'] = 360,  # that is basically the traveling range
        axis1['pos_step'] = 100,
        axis1['vel_min'] = 1,
        axis1['vel_max'] = 20,
        axis1['vel_step'] = 0.1,
        axis1['acc_min'] = None
        axis1['acc_max'] = None
        axis1['acc_step'] = None

        # assign the parameter container for x to a name which will identify it
        constraints[axis0['label']] = axis0
        constraints[axis1['label']] = axis1
        """
        pass

    @abstractmethod
    def move_rel(self,  axes_displacement):
        """ Moves stage in given direction (relative movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        A smart idea would be to ask the position after the movement.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abstractmethod
    def move_abs(self, axes_position):
        """ Moves stage to absolute position (absolute movement)

        @param dict param_dict: dictionary, which passes all the relevant
                                parameters, which should be changed. Usage:
                                 {'axis_label': <the-abs-pos-value>}.
                                 'axis_label' must correspond to a label given
                                 to one of the axis.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abstractmethod
    def get_pos(self, axes=None):
        """ Gets current position of the stage arms

        @param list param_list: optional, if a specific position of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                position is asked.

        @return dict: with keys being the axis labels and item the current
                      position.
        """
        pass

    @abstractmethod
    def abort(self, axes=None):
        """ Stops movement of the stage

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abstractmethod
    def home(self, axes=None):
        """ Calibrates the stage.

        @param dict param_list: param_list: optional, if a specific calibration
                                of an axis is desired, then the labels of the
                                needed axis should be passed in the param_list.
                                If nothing is passed, then all connected axis
                                will be calibrated.

        @return int: error code (0:OK, -1:error)

        After calibration the stage moves to home position which will be the
        zero point for the passed axis. The calibration procedure will be
        different for each stage.
        """
        pass

    @abstractmethod
    def get_status(self, axes=None):
        """ Get the status of the position

        @param list param_list: optional, if a specific status of an axis
                                is desired, then the labels of the needed
                                axis should be passed in the param_list.
                                If nothing is passed, then from each axis the
                                status is asked.

        @return dict: with the axis label as key and the status number as item.
        """
        pass