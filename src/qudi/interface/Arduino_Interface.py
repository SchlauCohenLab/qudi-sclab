# -*- coding: utf-8 -*-

"""
This file contains the Qudi Interface for a Arduino.

Copyright (c) 2021, the qudi developers. See the AUTHORS.md file at the top-level directory of this
distribution and on <https://github.com/Ulm-IQO/qudi-iqo-modules/>

This file is part of qudi.

Qudi is free software: you can redistribute it and/or modify it under the terms of
the GNU Lesser General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

Qudi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with qudi.
If not, see <https://www.gnu.org/licenses/>.
"""

from abc import abstractmethod
from qudi.core.module import Base


class ArduinoInterface(Base):
    """ This interface is used to manage and visualize a simple Arduino used for control of the pump ON/OFF signal.
    """

    @abstractmethod
    def on_activate(self):
        """ Initialize the Arduino connection """
        pass

    @abstractmethod
    def on_deactivate(self):
        """ Close the Arduino connection """
        pass

    @abstractmethod
    def set_pin_high(self):
        """ Set the pin to high (True) """
        pass

    def set_pin_low(self):
        """ Set the pin to low (False) """
        pass
