# -*- coding: utf-8 -*-

__all__ = ['MotorLogic']

from PySide2 import QtCore

from qudi.core.module import LogicBase
from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import RecursiveMutex

# qudi logic measurement modules must inherit qudi.core.module.LogicBase or other logic modules.
class MotorLogic(LogicBase):
    """ Logic module for interacting with the hardware motors.
    This logic has the same structure as the MotorInterface but supplies additional functionality:
        - motors can either be manipulated by index or by their names
        - signals are generated on state changes

    motor_logic:
        module.Class: 'motor_logic.MotorLogic'
        connect:
            actuator: actuator
    """

    # connector for one actuator, if multiple motors are needed use the MotorCombinerInterfuse
    motor = Connector(interface='MotorInterface')

    _scan_min = StatusVar('scan_min', default={})
    _scan_max = StatusVar('scan_max', default={})
    _scan_step = StatusVar('scan_step', default={})

    sigUpdatePosition = QtCore.Signal(dict)
    sigStartScanning = QtCore.Signal(dict)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = RecursiveMutex()

        self._watchdog_active = False
        self._watchdog_interval_ms = 0

    def on_activate(self):
        """ Activate module
        """
        self._constraints = self.motor().get_constraints()
        self._actual_position = self.motor().get_pos()
        self._axes = [axis for axis in self._constraints.keys()]
        for axis in self._axes:
            if axis not in self._scan_min.keys():
                self._scan_min[axis] = self._constraints[axis]['pos_min']
            if axis not in self._scan_min.keys():
                self._scan_max[axis] = self._constraints[axis]['pos_max']
            if axis not in self._scan_step.keys():
                self._scan_step[axis] = self._constraints[axis]['pos_step']


        self.sigStartScanning.connect(self._start_scanning)

    def on_deactivate(self):
        """ Deactivate module
        """
        self._watchdog_active = False

    def _start_scanning(self, axis):

        if self._position[axis] <= self._scan_max[axis]:
            return
        self.move_rel(self._scan_step[axis])
        self.sigStartScanning.emit(axis)

    def start_scan(self, axis):

        self.move_abs(self._scan_min[axis])
        self.sigStartScanning.emit(axis)

    @property
    def constraints(self):
        return self._constraints

    @property
    def status(self):
        return self.motor().get_status()

    @property
    def axes(self):
        return self._axes.copy()

    def move_abs(self, axes_displacement):
        """ Absolute displacement of the actuator axis.

        @return dict axes_displacement: Dictionary with axis name and target absolute displacement value

        @return dict pos: Dictionary with axis name and current position
        """
        if not isinstance(axes_displacement, dict):
            self.log.error("The input parameter is not a dictionary.")
            return

        for axis in axes_displacement.keys():

            pos_min, pos_max = self._constraints[axis]['pos_min'], self._constraints[axis]['pos_max']

            if not (pos_min <= axes_displacement[axis] <= pos_max):
                self.log.error("{} axis : the displacement is outside the allowed range.".format(axis))
                return

        self.motor().move_abs(axes_displacement)

        return self.motor().get_pos()

    def move_rel(self, axes_displacement):
        """ Relative displacement of the actuator axis.

        @return dict axes_displacement: Dictionary with axis name and target relative displacement value

        @return dict pos: Dictionary with axis name and current position
        """
        if not isinstance(axes_displacement, dict):
            self.log.error("The input parameter is not a dictionary.")
            return

        for axis in axes_displacement.keys():

            pos_min, pos_max = self._constraints[axis]['pos_min'], self._constraints[axis]['pos_max']

            if not (pos_min <= self.position[axis] + axes_displacement[axis] <= pos_max):
                self.log.error("{} axis : the displacement is outside the allowed range.".format(axis))
                return

        self.motor().move_rel(axes_displacement)

        return self.motor().get_pos()

    def get_position(self, axes_list):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self.motor().get_pos(axes_list)

    def get_status(self, axes_list):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self.motor().get_status(axes_list)

    @property
    def status(self):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self.motor().get_status()

    def home(self, axes_list):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self.motor().calibrate(axes_list)

    @property
    def position(self):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self.motor().get_pos()

    @property
    def scan_min(self):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self._scan_min

    @scan_min.setter
    def scan_min(self, scan_min):
        for axis, pos_min in scan_min.items():

            if not (self._constraints[axis]['pos_min'] <= pos_min <= self._constraints[axis]['pos_max']):
                self.log.error("{} axis : the scanning range is outside the allowed range.".format(axis))
                return

            self._scan_min[axis] = float(pos_min)

    @property
    def scan_max(self):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self._scan_max

    @scan_max.setter
    def scan_max(self, scan_max):

        for axis, pos_max in scan_max.items():
            if not (self._constraints[axis]['pos_min'] <= pos_max <= self._constraints[axis]['pos_max']):
                self.log.error("{} axis : the scanning range is outside the allowed range.".format(axis))
                return

            self._scan_max[axis] = float(pos_max)

    @property
    def scan_step(self):
        """ Current position of the actuator axis.

        @return dict pos: Dictionary with axis name and current position
        """
        return self._scan_step

    @scan_min.setter
    def scan_step(self, scan_step):

        for axis, step in scan_step.items():
            if not (self._constraints[axis]['pos_step'] <= step):
                self.log.error("{} axis : the scanning step is below the minimum displacement step.".format(axis))
                return

            self._scan_step[axis] = float(step)

    @property
    def velocity(self):
        """ Current velocity of the actuator axis.

        @return dict velocity: Dictionary with axis name and current velocity
        """
        return self.motor().get_velocity()

    @velocity.setter
    def velocity(self, velocity):
        """ Current velocity of the actuator axis.

        @return dict velocity: Dictionary with axis name and target velocity

        @return dict velocity: Dictionary with axis name and current velocity
        """
        if not isinstance(velocity, dict):
            self.log.error("The input parameter is not a dictionary.")
        current_vel = self.velocity
        for axis in self.motor().get_axis():
            vel_min, vel_max = self._constraints[axis]['vel_min'], self._constraints[axis]['vel_max']
            if not (vel_min <= current_vel[axis]+velocity[axis] <= vel_max):
                self.log.error("{} axis : the velocity is outside the allowed range.".format(axis))
        self.motor().set_velocity(velocity)
        return self.motor().get_velocity()
