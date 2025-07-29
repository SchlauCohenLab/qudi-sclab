# -*- coding: utf-8 -*-

__all__ = ['TACameraLogic']

from PySide2 import QtCore

from qudi.core.module import LogicBase
from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import RecursiveMutex
import threading

# qudi logic measurement modules must inherit qudi.core.module.LogicBase or other logic modules.
class TACameraLogic(LogicBase):
    """ Logic module for interacting with the camera to display TA spectrum


    actuator_logic:
        module.Class: 'TA_camera_logic.TACameraLogic'
        connect:
            camera: PCIe_1430_camera
            trigger: : Arduino_trigger
    """

    camera = Connector(interface='CameraInterface')
    trigger = Connector(interface='ArduinoInterface')

    spectrum_acquired = QtCore.Signal(object,object) # TA spectrum and white light

    def on_activate(self):
        self.running = False
        self.log.info(f'CameraLogic activated with n_frames')

    def set_nframes(self, n):
        """Set the number of frames to acquire."""
        try:
            self.camera().set_nframe(n)
        except Exception:
            raise self.log.info(f'Error when setting n_frames to {n}')
        
    def get_nframes(self):
        return self.camera().get_nframe()
    
    def start_acquisition(self):
        if not self.running:
            self.camera().start_live_acquisition()
            self.running = True
            threading.Thread(target=self._acquisition_loop, daemon=True).start()
    
    def stop_acquisition(self):
        self.running = False
        self.camera().stop_live_acquisition()

    def _acquisition_loop(self):
        while self.running:

            data = self.camera.get_acquired_data()
            self.trigger().set_pin_high()
            pump_on = data[::2].mean(axis=0)
            pump_off = data[1::2].mean(axis=0)
            self.trigger().set_pin_low()


            ta_spectrum = pump_on - pump_off

            self.last_wl = pump_off
            self.last_ta = ta_spectrum

            self.spectrum_acquired.emit(self.last_ta ,self.last_wl)

    def get_last_spectra(self):
        return getattr(self, 'last_ta', None), getattr(self, 'last_wl', None) 
