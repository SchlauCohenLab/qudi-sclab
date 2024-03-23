# -*- coding: utf-8 -*-

__all__ = ['SlowCounterInterface']

from abc import abstractmethod
from enum import Enum
from qudi.core.module import Base


class SlowCounterInterface(Base):
    """ Define the controls for a slow counter.

    A slow counter is a measuring device that measures with a precise frequency one or multiple physical quantities.

    An example is a device that counts photons in real time with a given frequency.

    The main idea of such a device is that the hardware handles the timing, and measurement of one or multiple
    time varying quantities. The logic will periodically (but with imprecise timing) poll the hardware for the new
    reading, not knowing if there is one, multiple or none.
    """

    @abstractmethod
    def get_constraints(self):
        """ Retrieve the hardware constrains from the counter device.

        @return (SlowCounterConstraints): object with constraints for the counter

        The constrains are defined as a SlowCounterConstraints object, defined at  the end of this file
        """
        pass

    @abstractmethod
    def set_up_clock(self, clock_frequency=None, clock_channel=None):
        """ Set the frequency of the counter by configuring the hardware clock

        @param (float) clock_frequency: if defined, this sets the frequency of the clock
        @param (string) clock_channel: if defined, this is the physical channel of the clock
        @return int: error code (0:OK, -1:error)

        TODO: Should the logic know about the different clock channels ?
        """
        pass

    @abstractmethod
    def set_up_counter(self,
                       counter_channels=None,
                       sources=None,
                       clock_channel=None,
                       counter_buffer=None):
        """ Configures the actual counter with a given clock.

        @param list(str) counter_channels: optional, physical channel of the counter
        @param list(str) sources: optional, physical channel where the photons
                                   photons are to count from
        @param str clock_channel: optional, specifies the clock channel for the
                                  counter
        @param int counter_buffer: optional, a buffer of specified integer
                                   length, where in each bin the count numbers
                                   are saved.

        @return int: error code (0:OK, -1:error)

        There need to be exactly the same number sof sources and counter channels and
        they need to be given in the same order.
        All counter channels share the same clock.
        """
        pass

    @abstractmethod
    def get_counter(self, samples=None):
        """ Returns the current counts per second of the counter.

        @param int samples: if defined, number of samples to read in one go

        @return numpy.array((n, uint32)): the measured quantity of each channel
        """
        pass

    @abstractmethod
    def get_counter_channels(self):
        """ Returns the list of counter channel names.

        @return list(str): channel names

        Most methods calling this might just care about the number of channels, though.
        """
        pass

    @abstractmethod
    def close_counter(self):
        """ Closes the counter and cleans up afterwards.

        @return int: error code (0:OK, -1:error)
        """
        pass

    @abstractmethod
    def close_clock(self):
        """ Closes the clock and cleans up afterwards.

        @return int: error code (0:OK, -1:error)

        TODO: This method is very hardware specific, it should be deprecated
        """
        pass


class CountingMode(Enum):
    """
    TODO: Explain what are the counting mode and how they are used
    """
    CONTINUOUS = 0
    GATED = 1
    FINITE_GATED = 2


class SlowCounterConstraints:

    def __init__(self):
        # maximum numer of possible detectors for slow counter
        self.max_detectors = 0
        # frequencies in Hz
        self.min_count_frequency = 5e-5
        self.max_count_frequency = 5e5
        # TODO: add CountingMode enums to this list in instances
        self.counting_mode = []