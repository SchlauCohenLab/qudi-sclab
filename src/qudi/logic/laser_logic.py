# -*- coding: utf-8 -*-

__all__ = ['LaserLogic']

from PySide2 import QtCore

from qudi.core.module import LogicBase
from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.interface.laser_interface import ShutterState, LaserState


# qudi logic measurement modules must inherit qudi.core.module.LogicBase or other logic modules.
class TemplateLogic(LogicBase):
    """ This is a simple template logic measurement module for qudi.

    Example config that goes into the config file:

    example_logic:
        module.Class: 'template_logic.TemplateLogic'
        options:
            increment_interval: 2
        connect:
            template_hardware: dummy_hardware
    """

    # Declare signals to send events to other modules connecting to this module
    sigCounterUpdated = QtCore.Signal(int)  # update signal for the current integer counter value

    # Declare static parameters that can/must be declared in the qudi configuration
    _increment_interval = ConfigOption(name='increment_interval', default=1, missing='warn')

    # Declare status variables that are saved in the AppStatus upon deactivation of the module and
    # are initialized to the saved value again upon activation.
    _counter_value = StatusVar(name='counter_value', default=0)

    # Declare connectors to other logic modules or hardware modules to interact with
    _template_hardware = Connector(name='template_hardware',
                                   interface='TemplateInterface',
                                   optional=True)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()  # Mutex for access serialization

    def on_activate(self) -> None:
        # Check if _increment_interval is not too small (lower boundary is 1.5 * trigger_time)
        assert self._increment_interval >= 1.5 * self._template_hardware().trigger_time, \
            'increment_interval must be >= 1.5 * <hardware trigger time>'
        # Set up a Qt timer to send periodic signals according to _increment_interval
        self.__timer = QtCore.QTimer(parent=self)
        self.__timer.setInterval(1000 * self._increment_interval)  # Interval in milliseconds
        self.__timer.setSingleShot(False)
        # Connect timeout signal to increment slot
        self.__timer.timeout.connect(lambda: self.add_to_counter(1), QtCore.Qt.QueuedConnection)
        # Start timer
        self.__timer.start()

    def on_deactivate(self) -> None:
        # Stop timer and delete
        self.__timer.stop()
        self.__timer.timeout.disconnect()
        self.__timer = None

    @property
    def counter_value(self) -> int:
        with self._mutex:
            return self._counter_value

    def add_to_counter(self, value: int) -> None:
        if value != 0:
            with self._mutex:
                if value > 0:
                    hardware = self._template_hardware()
                    for i in range(value):
                        hardware.send_trigger()
                        self._counter_value += 1
                        self.sigCounterUpdated.emit(self._counter_value)
                else:
                    self._counter_value += value
                    self.sigCounterUpdated.emit(self._counter_value)

    def reset_counter(self) -> None:
        with self._mutex:
            self._counter_value = 0
            self.sigCounterUpdated.emit(self._counter_value)


class LaserLogic(LogicBase):
    """ Logic module to control a laser.

    laserlogic:
        module.Class: 'laser_logic.LaserLogic'
        connect:
            laser: 'mylaser'
    """

    laser = Connector(interface='LaserInterface')
    _wavelength = StatusVar('wavelength', None)
    _power_setpoint = StatusVar('power_setpoint', None)

    sigUpdateSettings = QtCore.Signal()

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def on_activate(self):
        """ Activate module.
        """
        self._constraints = self.laser().get_constraints()

        if self._wavelength == None:
            self._wavelength = self.laser().get_wavelength()
        else:
            self.wavelength = self._wavelength

        if self._power_setpoint == None:
            self._power_setpoint = self.laser().get_power_setpoint()
        else:
            self.power_setpoint = self._power_setpoint

        self._shutter_state = self.laser().get_shutter_state()
        self._laser_state = self.laser().get_laser_state()


    def on_deactivate(self):
        """ Deactivate module.
        """
        return

    @property
    def laser_state(self):
        """Getter method to know the laser state (ON or OFF).

        @return (LaserState) laser_state: laser state (ON or OFF).
        """
        self._laser_state = self.laser().get_laser_state()
        return self._laser_state

    @laser_state.setter
    def laser_state(self, laser_state):
        """Setter method to control the laser state (ON or OFF).

        @param (LaserState) laser_state: laser state (ON or OFF).
        """
        if self._laser_state == laser_state:
            return
        if isinstance(laser_state, str) and laser_state in LaserState.__members__:
            laser_state = LaserState[laser_state]
        if not isinstance(laser_state, LaserState):
            self.log.error("Laser state parameter do not match with laser states of the laser.")
            return
        self.laser().set_laser_state(laser_state)
        self._laser_state = self.laser().get_laser_state()

    @property
    def wavelength(self):
        """Getter method to know the laser wavelength in m if tunable.

        @return (float) wavelength: laser wavelength in m.
        """
        self._wavelength = self.laser().get_wavelength()
        return self._wavelength

    @wavelength.setter
    def wavelength(self, wavelength):
        """Setter method to control the laser wavelength in m if tunable.

        @param (float) wavelength: laser wavelength in m.
        """
        if not self._constraints.tunable_wavelength:
            self.log.warning('The laser hardware module is not wavelength tunable. The wavelength cannot be changed.')
            return
        wavelength = float(wavelength)
        wavelength_min, wavelength_max = self._constraints.wavelength_range
        if not wavelength_min <= wavelength < wavelength_max:
            self.log.error('Wavelength value is not correct : it must be in range {} to {} '
                           .format(wavelength_min, wavelength_max))
            return
        self.laser().set_wavelength(wavelength)
        self._wavelength = self.laser().get_wavelength()
        self.sigUpdateSettings.emit()

    @property
    def power(self):
        """Getter method to know the laser power in W if tunable.

        @return (float) power: laser power in W.
        """
        return self.laser().get_power()

    @property
    def power_setpoint(self):
        """Getter method to know the laser power setpoint in W if tunable.

        @return (float) power setpoint: laser power setpoint in W.
        """
        self._power_setpoint = self.laser().get_power_setpoint()
        return self._power_setpoint

    @power_setpoint.setter
    def power_setpoint(self, power_setpoint):
        """Setter method to control the laser power_setpoint in W if tunable.

        @param (float) power_setpoint: laser power_setpoint in W.
        """
        if not self._constraints.tunable_power:
            self.log.warning('The laser hardware module is not power tunable. The laser power cannot be changed.')
            return
        power_setpoint = float(power_setpoint)
        power_min, power_max = self._constraints.power_range
        if not power_min <= power_setpoint < power_max:
            self.log.error('Power value is not correct : it must be in range {} to {} '
                           .format(power_min, power_max))
            return
        self.laser().set_power_setpoint(power_setpoint)
        self._power_setpoint = self.laser().get_power_setpoint()
        self.sigUpdateSettings.emit()

    @property
    def shutter_state(self):
        """Getter method to control the laser shutter if available.

        @return (ShutterState) shutter_state: shutter state (OPEN/CLOSED/AUTO).
        """
        if not self._constraints.has_shutter:
            self.log.error("No shutter is available in your hardware ")
            return
        self._shutter_state = self.laser().get_shutter_state()
        return self._shutter_state

    @shutter_state.setter
    def shutter_state(self, shutter_state):
        """Setter method to control the laser shutter if available.

        @param (ShutterState) shutter_state: shutter state (OPEN/CLOSED/AUTO).
        """
        if not self._constraints.has_shutter:
            self.log.error("No shutter is available in your hardware ")
            return
        if self._shutter_state == shutter_state:
            return
        if isinstance(shutter_state, str) and shutter_state in ShutterState.__members__:
            shutter_state = ShutterState[shutter_state]
        if not isinstance(shutter_state, ShutterState):
            self.log.error("Shutter state parameter do not match with shutter states of the camera ")
            return
        self.laser().set_shutter_state(shutter_state)
        self._shutter_state = self.laser().get_shutter_state()
