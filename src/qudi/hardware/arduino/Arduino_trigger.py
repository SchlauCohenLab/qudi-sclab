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
from qudi.core.configoption import ConfigOption
import serial
import time

class ArduinoTrigger(ArduinoInterface): 
    """ Arduino hardware module for triggering pump on/off signal.
        Example config for copy-paste:

    Arduino_trigger:
        module.Class: 'arduino.arduino_trigger.ArduinoTrigger'
        options:
            port: 'COM5'
            baudrate: 115200
            Pin_ONOFF: 13 #Pin used to communicate with the digital output of the arduino to the digital discovery
        """
    
    _port = ConfigOption('port', missing='error')
    _baudrate = ConfigOption('baudrate', default=115200)
    _pin = ConfigOption('Pin_ONOFF',  missing='error')

    # Tu pourrais laisser l'utilisateur changer le pin à la volée plutôt que de le fixer dans le fichier de config.

    def on_activate(self):
        """ Initialize the Arduino connection """
        try:
            self._arduino = serial.Serial(self._port, self._baudrate, timeout=1)
            time.sleep(2)
            if self._arduino.is_open:
                self.log.info(f"Arduino connected successfully on {self._port}")
            else:
                self.log.error(f"Arduino connection failed on {self._port}")
        except serial.SerialException as e:
            self.log.error(f"Failed to connect to Arduino: {e}")
            raise
    
    def on_deactivate(self):
        """ Close the Arduino connection """
        if self.serial.is_open:
            self.serial.close()
            self.log.info(f"Closed connection to Arduino on port {self._port}")
        else:
            self.log.error('Connection to Arduino is already closed.')

    # Plus simplement tu peux faire un set_pin(pin, value) qui prend en argument le pin et la valeur à mettre.
    def set_pin_high(self):
        """ Set the pin to high (True) """
        if self._arduino.is_open:
            self._arduino.write(f'{self._pin}H\n'.encode())
            self.log.info(f'Set pin {self._pin} to True')
        else:
            self.log.error('Arduino connection is not open.')

    def set_pin_low(self):
        """ Set the pin to low (False) """
        if self._arduino.is_open:
            self._arduino.write(f'{self._pin}L\n'.encode())
            self.log.info(f'Set pin {self._pin} to False')
        else:
            self.log.error('Arduino connection is not open.')
