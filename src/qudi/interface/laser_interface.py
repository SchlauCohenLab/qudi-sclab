# -*- coding: utf-8 -*-

__all__ = ['LaserInterface']

from abc import abstractmethod
from enum import Enum
from qudi.core.module import Base

class ShutterState(Enum):
    CLOSED = 0
    OPEN = 1

class LaserState(Enum):
    OFF = 0
    ON = 1

class Constraints:
    """ Class defining formally the hardware constraints """
    def __init__(self):
        self.tunable_power = None        # If the laser power is tunable
        self.tunable_wavelength = None   # If the laser wavelength is tunable
        self.has_shutter = None          # If the laser has shutter
        self.power_range = None          # The laser power range min and max if tunable
        self.wavelength_range = None     # The laser wavelength range min and max if tunable

class LaserInterface(Base):
    """ This is a simple template hardware interface for qudi.
    """
    
    @abstractmethod
    def get_constraints(self):
        """ Returns all the fixed parameters of the hardware which can be used by the logic.

        @return (Constraints): An object of class Constraints containing all fixed parameters of the hardware
        """
        pass

    @abstractmethod
    def set_laser_state(self, laser_state):
        """Setter method to control the laser state (ON or OFF).

        @param (LaserState) laser_state: laser state (ON or OFF).
        """
        pass

    @abstractmethod
    def get_laser_state(self):
        """Getter method to know the laser state (ON or OFF).

        @return (LaserState) laser_state: laser state (ON or OFF).
        """
        pass

    @abstractmethod
    def set_wavelength(self, wavelength):
        """Setter method to control the laser wavelength in m if tunable.

        @param (float) wavelength: laser wavelength in m.
        """
        pass

    @abstractmethod
    def get_wavelength(self):
        """Getter method to know the laser wavelength in m if tunable.

        @return (float) wavelength: laser wavelength in m.
        """
        pass

    @abstractmethod
    def set_power_setpoint(self, power):
        """Setter method to control the laser power setpoint in W if tunable.

        @param (float) power setpoint: laser power in W.
        """
        pass

    @abstractmethod
    def get_power_setpoint(self):
        """Getter method to know the laser power setpoint in W if tunable.

        @return (float) power setpoint: laser power setpoint in W.
        """
        pass

    @abstractmethod
    def get_power(self):
        """Getter method to know the laser current power in W if tunable.

        @return (float) power: laser power in W.
        """
        pass

    @abstractmethod
    def set_shutter_state(self, shutter_state):
        """Setter method to control the laser shutter if available.

        @param (ShutterState) shutter_state: shutter state (OPEN/CLOSED/AUTO).
        """
        pass

    @abstractmethod
    def get_shutter_state(self):
        """Getter method to control the laser shutter if available.

        @return (ShutterState) shutter_state: shutter state (OPEN/CLOSED/AUTO).
        """
        pass
