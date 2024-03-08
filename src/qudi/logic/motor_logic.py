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
        options:
            watchdog_interval: 1  # optional
            autostart_watchdog: True  # optional
        connect:
            motor: motor
    """

    # connector for one motor, if multiple motors are needed use the MotorCombinerInterfuse
    motor = Connector(interface='MotorInterface')

    # declare config options :
    _watchdog_interval = ConfigOption(name='watchdog_interval', default=1.0, missing='nothing')
    _autostart_watchdog = ConfigOption(name='autostart_watchdog', default=False, missing='nothing')

    # declare status variables (logic attribute) :
    _move_absolute = StatusVar('move_absolute', True)

    sigUpdatePosition = QtCore.Signal(dict)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = RecursiveMutex()

        self._watchdog_active = False
        self._watchdog_interval_ms = 0

    def on_activate(self):
        """ Activate module
        """
        self.motor_constraints = self.motor().get_constraints()
        self._actual_position = self.motor().get_pos()
        self._watchdog_interval_ms = int(round(self._watchdog_interval * 1000))

    def on_deactivate(self):
        """ Deactivate module
        """
        self._watchdog_active = False

    @property
    def actual_position(self):
        """ Actual position of the motor axis.

        @return dict pos: Dictionary with axis name and pos in deg
        """
        self._actual_position = self.motor().get_pos()
        return self._actual_position

    def move(self, displacement):
        """ Actual position of the motor axis.

        @return dict pos: Dictionary with axis name and pos in deg
        """
        if self._move_absolute:
            self.motor().move_abs(displacement)
        else:
            self.motor().move_rel(displacement)

    @property
    def velocity(self):
        """ Actual position of the motor axis.

        @return dict velocity: Dictionary with axis name and target velocity
        """
        return self.motor().get_velocity()

    @velocity.setter
    def velocity(self, velocity):
        """ Actual position of the motor axis.

        @param dict param_dict: Dictionary with axis name and target velocity
        """
        self.motor().set_velocity(velocity)

    @property
    def move_absolute(self):
        """ Absolute or relative motor axis displacement.

        @return bool : if the motor axis displacement is absolute.
        """
        return self._move_absolute

    @move_absolute.setter
    def move_absolute(self, move_absolute):
        """ Actual position of the motor axis.

        @param bool : if the motor axis displacement is absolute.
        """
        self._move_absolute = move_absolute
