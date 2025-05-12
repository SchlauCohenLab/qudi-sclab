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
from qudi.hardware_interface.camera_interface import BaseCamera
import logging

logger = logging.getLogger(__name__)

class PCIe1430Camera(BaseCamera):
    """ Hardware class for EV2

    Example config for copy-paste:

    PCIe-1430:
        module.Class: 'camera.PCIe-1430_camera.PCIe1430Camera'
        options:
            dll_location: 'path/to/dlls'
            cam_name: name identifying the camera 


    """

    _dll_location = ConfigOption('dll_location', None) 
    """"
    the DLL is automatically added to the System32 folder,where pylablib looks for it by default. If the DLL is located elsewhere,
    the path has to be specified. Otherzise, can be left as None. 
    """
    _name = ConfigOption('camera_name',default = 'cam1')
    _default_nframes = ConfigOption('nframes', default=5000)

    _nframe = _default_nframes
    _live = True
    cam = None




    def on_activate(self):
        """ Initialisation performed during activation of the module."""
        if self._dll_location:
            pll.par["devices/dlls/niimaq"] = self._dll_location
        try:
            self.cam = IMAQ.IMAQCamera(self._name)
            self._width, self._height = self.cam.get_detector_size()
        except Exception as e:
            logger.error(f"Failed to start NI IMAQ camera: {e}")
            raise

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        if self.cam:
            self.cam.close()
            self.cam = None
            logger.info("NI IMAQ camera stopped")
        else:
            logger.info("Camera is not activated, nothing to stop")

    def get_name(self):
        """ Retrieve an identifier of the camera that the GUI can print

        @return string: name for the camera
        """
        return self.cam.get_device_info()


    def get_size(self):
        """ Retrieve size of the image in pixel

        @return tuple: Size (width, height)
        """
        return  self._width, self._height


    def support_live_acquisition(self):
        """ Return whether or not the camera can take care of live acquisition

        @return bool: True if supported, False if not
        """
        return self._live


    def start_live_acquisition(self):
        """ Start a continuous acquisition

        @return bool: Success ?
        """
        self.cam.start_acquisition(mode='sequence', nframes=self._nframe) 
        self.cam.start_acquisition() 
        return self.cam.acquisition_in_progress() 


    def start_single_acquisition(self):
        """ Start a single snao acquisition

        @return bool: Success ?
        """
        self.cam.setup_acquisition(mode='snap', nframes=self._frames)
        self.cam.start_acquisition()
        return self.acquisition_in_progress() 

    def stop_acquisition(self):
        """ Stop/abort live or single acquisition

        @return bool: Success ?
        """
        if self.cam.acquisition_in_progress():
            self.cam.stop_acquisition()
            logger.debug("Acquisition aborted")
        return  not self.acquisition_in_progress()


    def get_acquired_data(self):
        """ Return an array of last acquired image.

        @return numpy array: image data in format [[row],[row]...]

        Each pixel might be a float, integer or sub pixels
        """
        return self.cam.read_newest_image()


    def set_nframe(self, nframe):
        """ Set the number of frames per steps

        @param float frame: desired number of frame per step

        @return float: setted new number of frame 
        """
        if not float(nframe) or nframe <= 0:
            self.log.warning("Give me some positive float")
            return self._nframe
        self._nframe = nframe
        logger.info("Number of frames set to %s", self._nframe)
        return self._nframe
        

    def get_nframe(self):
        """ Get the number of frames per steps

        @return float number of framws
        """
        return self. _nframe

    def get_ready_state(self):
        """ Is the camera ready for an acquisition ?

        @return bool: ready ?
        """
        return self.cam is not None