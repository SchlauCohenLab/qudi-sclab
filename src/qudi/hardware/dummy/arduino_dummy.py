"""
This hardware module implement the Arduino trigerring to have the pump on/off signal.
It helps synchronize acquisition with the chopper TTL signal that allows to differentiate between pump on and pump off​.
See <https://www.dropbox.com/scl/fi/n3itifrhxoqmqarh84ev4/24-08-09_TA_Trigger_Documentation.pptx?cloud_editor=powerpoint&dl=0> for more infotmation on the implemented triggerign scheme. 
---

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
from qudi.interface.Arduino_Interface import ArduinoInterface


class ArduinoDummy(ArduinoInterface): 
    """ Arduino hardware module for dummy.
        Example config for copy-paste:

    arduino_dummy:
        module.Class: 'dummy.arduino_dummy.ArduinoDummy'  

    """

    def on_activate(self):
        """ Initialisation performed during activation of the module."""
    pass

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
    pass

    def set_pin_high(self):
        """ Set the pin to high (True) """
        return 1

    def set_pin_low(self):
        return 0
    
    def test_connection(self):
            raise self.log.error('Aduino connection is not open.')
