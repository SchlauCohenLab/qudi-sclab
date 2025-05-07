# -*- coding: utf-8 -*-

# FIXME: This module is obviously taken from someone else and altered without attribution.
"""
This hardware module implement the camera spectrometer interface to use an IMAQ Camera.
It use a dll to interface with instruments via USB (only available physical interface)
This module does aim at replacing Solis.

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

from enum import Enum
from ctypes import *
import numpy as np
from pylablib.devices import IMAQ
import pylablib as pll
from qudi.core.configoption import ConfigOption
from qudi.interface.camera_interface import CameraInterface





class ReadMode(Enum):
    FVB = 0
    MULTI_TRACK = 1
    RANDOM_TRACK = 2
    SINGLE_TRACK = 3
    IMAGE = 4


class AcquisitionMode(Enum):
    SNAP = 1
    SEQUENCE = 2


class TriggerMode(Enum):
    INTERNAL = 0
    EXTERNAL = 1
    EXTERNAL_START = 6
    EXTERNAL_EXPOSURE = 7
    SOFTWARE_TRIGGER = 10
    EXTERNAL_CHARGE_SHIFTING = 12


class PCIe1430Camera(CameraInterface):
    """ Hardware class for Andors Ixon Ultra 897

    Example config for copy-paste:

    PCIe-1430:
        module.Class: 'camera.PCIe-1430_camera.PCIe1430Camera'
        options:
            dll_location: 'path/to/dlls'
            cam_name: name identifying the camera 


    """

    _dll_location = ConfigOption('dll_location', missing='warning')
    _name = ConfigOption('cam_name', missing='error')

    """
    Interfance with the camera requires imaq.dll, which is installed with the 
    freely available Vision Acquisition Software, which also includes all the necessary drivers. 
    After installation, the DLL is automatically added to the System32 folder,
     where pylablib looks for it by default. If the DLL is located elsewhere,
    the path can be specified using the library parameter devices/dlls/niimaq:
    """

    #pll.par["devices/dlls/niimaq"] =_dll_location
    cam1 = IMAQ.IMAQCamera(_name)  # Put it in on_acitivate  


    def on_activate(self):
        """ Initialisation performed during activation of the module.
         """
        # self.cam.SetAcquisitionMode(1)  # single
        # self.cam.SetTriggerMode(0)  # internal
        # self.cam.SetCoolerMode(0)  # Returns to ambient temperature on ShutDown
        # self.set_cooler_on_state(self._cooler_on)
        # self.set_exposure(self._exposure)
        # self.set_setpoint_temperature(self._temperature)
        self.dll = cdll.LoadLibrary(self._dll_location)
        self.dll.Initialize()
        nx_px, ny_px = c_int(), c_int()
        self._get_detector(nx_px, ny_px)
        self._width, self._height = nx_px.value, ny_px.value
        self._set_read_mode(self._read_mode)
        self._set_trigger_mode(self._trigger_mode)
        self._set_exposuretime(self._exposure)
        self._set_acquisition_mode(self._acquisition_mode)

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self.stop_acquisition() #also break the connection and so on

    def get_name(self):
        """ Retrieve an identifier of the camera that the GUI can print

        @return string: name for the camera
        """
        return cam1.get_device_info()


    def get_size(self):
        """ Retrieve size of the image in pixel

        @return tuple: Size (width, height)
        """
        return cam1.get_detector_size()


    def support_live_acquisition(self):
        """ Return whether or not the camera can take care of live acquisition

        @return bool: True if supported, False if not
        """
        return True


    def start_live_acquisition(self):
        """ Start a continuous acquisition

        @return bool: Success ?
        """
        cam1.start_acquisition(mode='sequence', nframes=100)  # ask Adrien how it should be done here 
        return cam1.acquisition_in_progress() 


    def start_single_acquisition(self):
        """ Start a single acquisition

        @return bool: Success ?
        """
        self.setup_acquisition(mode='snap', nframes=self._frames)
        pass self.acquisition_in_progress() # ask Adrien how it should be done here 


    def stop_acquisition(self):
        """ Stop/abort live or single acquisition

        @return bool: Success ?
        """
        if cam1.acquisition_in_progress():
            cam1.stop_acquisition()

        return  not cam1.acquisition_in_progress()


    def get_acquired_data(self):
        """ Return an array of last acquired image.

        @return numpy array: image data in format [[row],[row]...]

        Each pixel might be a float, integer or sub pixels
        """
        cam1.
        return cam1.read_newest_image()


    def set_exposure(self, exposure):
        """ Set the exposure time in seconds

        @param float exposure: desired new exposure time

        @return float: setted new exposure time
        """
        if not float(exposure):
            self.log.warning("Give me some float")
            return
        
        self._frames = exposure//dt
        pass


    def get_exposure(self):
        """ Get the exposure time in seconds

        @return float exposure time
        """
        pass


    def set_gain(self, gain):
        """ Set the gain

        @param float gain: desired new gain

        @return float: new exposure gain
        """
        pass


    def get_gain(self):
        """ Get the gain

        @return float: exposure gain
        """
        pass


    def get_ready_state(self):
        """ Is the camera ready for an acquisition ?

        @return bool: ready ?
        """
        pass