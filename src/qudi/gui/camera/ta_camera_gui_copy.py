__all__ = ['CameraGuiModule'] 
import sys
import os

from qudi.core.module import GuiBase
from qtpy import QtWidgets
import pyqtgraph as pg
from qudi.core.connector import Connector
import numpy as np
from lmfit import Model

def linear(x,a,b):
    return a*x+b

class MainWindow(QtWidgets.QMainWindow):
    """ GUI module to continuously display TA and reference spectra. """

    def __init__(self,camera_logic):
        super().__init__()
        self.logic = camera_logic

        self._pixel = np.arange(0, 2048)
        self._x = self._pixel

        # Create the main Qt windget 
        self.w = QtWidgets.QWidget()
        self.setCentralWidget(self.w)
        self.w.setWindowTitle('TA test')




        #Create TA spectrum display widget
        self.plot_widget_ta = pg.PlotWidget(title="TA Spectrum")
        self.plot_widget_ta.setLabel('left', 'Intensity (OD)')
        self.ta_line = self.plot_widget_ta.plot([], pen='y')


        # White light reference display widget
        self.plot_widget_ref = pg.PlotWidget(title="White Light Reference")
        self.plot_widget_ref.setLabel('left', 'Intensity')
        self.ref_line = self.plot_widget_ref.plot([], pen='c')

        # Start/stop button
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self._toggle_acquisition)
        self.start_button.setMinimumHeight(40)


        # Change the number of frame - spinbox
        self.nframe_label = QtWidgets.QLabel("Number of frames per step:")
        self.nframe_spin = QtWidgets.QSpinBox()
        self.nframe_spin.setMinimum(1)
        self.nframe_spin.setMaximum(10000)
        self.nframe_spin.setValue(self.logic().get_nframes())
        self.nframe_spin.editingFinished.connect(self._update_nframes) #Chnage via main box
        self.nframe_spin.valueChanged.connect(self._on_spinbox_clicked) #Change via the spin box
        self.nframe_spin.setMinimumHeight(40)

        # Load the calibration file
        self.load_calibration_button = QtWidgets.QPushButton("Load Calibaration")
        self.load_calibration_button.clicked.connect(self._calibration)
        self.load_calibration_button.setMinimumHeight(40)

        #Switch between pixel/wavelength
        self.wl_button = QtWidgets.QPushButton("Change to wavelength")
        self.wl_button.clicked.connect(self._toggle_wavelength)
        self.wl_button.setMinimumHeight(40)

        #Create a grid layout to manage wigetts size abd position
        self.layout = QtWidgets.QGridLayout()
        self.w.setLayout(self.layout)

        #Create a sublayer to have the label and spin close to each others
        self.nframe_layout = QtWidgets.QHBoxLayout() 
        self.nframe_layout.addWidget(self.nframe_label) 
        self.nframe_layout.addWidget(self.nframe_spin)
        self.nframe_widget = QtWidgets.QWidget()
        self.nframe_widget.setLayout(self.nframe_layout)

        #Create a sublayer for the calibration 
        self.calibration_layout = QtWidgets.QHBoxLayout() 
        self.calibration_layout.addWidget(self.load_calibration_button)
        self.calibration_layout.addWidget(self.wl_button)
        self.calibration_widget = QtWidgets.QWidget()
        self.calibration_widget.setLayout(self.calibration_layout)

        # Add widgets to the layout
        self.layout.addWidget(self.plot_widget_ta,0,0,1,1) # upper left
        self.layout.addWidget(self.plot_widget_ref,0,1,1,2) # upper right
        self.layout.addWidget(self.start_button,1,0) # lower left
        self.layout.addWidget(self.nframe_widget,1,1) # lower center
        self.layout.addWidget(self.calibration_widget,1,2) # lower right

    def _toggle_acquisition(self):
        if self.start_button.text() == "Start":
            self.logic().start_acquisition()
            self.start_button.setText("Stop")
        else:
            self.logic().stop_acquisition()
            self.start_button.setText("Start")

    def _toggle_wavelength(self):
        if self.wl_button.text() == "Change to wavelength":
            if  hasattr(self, '_wavelength'):       
                self._x = self._wavelength
                self.wl_button.setText("Change to pixels")
            else:
                QtWidgets.QMessageBox.warning(self.w, "Warning", "Please load a calibration file first")
                return
        else:
            self._x = self._pixel
            self.wl_button.setText("Change to wavelength")

    def _update_nframes(self):
        if self.start_button.text() == "Start": #Camera is not acquiering 
            new_val = self.nframe_spin.value()
            self.logic().set_nframes(new_val)
        else:
            self.logic().stop_acquisition()
            new_val = self.nframe_spin.value()
            self.logic().set_nframes(new_val)
            self.logic().start_acquisition()

    def _on_spinbox_clicked(self, value):
    # Only update if changed via arrow buttons (not while typing)
        if self.nframe_spin.hasFocus():
            return # Do nothing yet — wait for editingFinished
        self._update_nframes()

    def _update_plot(self, ta_spectrum, reference):
        self.ta_line.setData(self._x , ta_spectrum)
        self.ref_line.setData(self._x , reference)

    def _calibration(self):
        self.path,_ = QtWidgets.QFileDialog.getOpenFileName(self.w, "Load calibration file",'', "Text Files (*.txt)")
        if self.path:
            self.data = np.loadtxt(self.path, delimiter = ' ')
            self.pxl_cal = self.data[:,0]
            self.wl_cal = self.data[:,1] 
            self.linear_model = Model(linear)
            self.a_guess, self.b_guess = np.polyfit(x= self.pxl_cal, y=self.wl_cal,deg=1)
            self.params = self.linear_model.make_params(a = self.a_guess, b= self.b_guess)
            self.result_fit = self.linear_model.fit(self.wl_cal, x= self.pxl_cal, params = self.params)
            self._wavelength = self.result_fit.params['a'].value*self._pixel + self.result_fit.params['b'].value


class CameraGuiModule(GuiBase):
    """ GUI module for interacting with a camera via TACameraLogic.
    
    Example config for copy-paste:

    camera_gui:
        module.Class: 'camera.ta_camera_gui.CameraGuiModule'
        connect:
            camera_logic: 'TA_logic'
    """
    camera_logic = Connector(interface='TACameraLogic')
    

    def on_activate(self):
        self.log.info("Activating Camera_gui")
        self.window = MainWindow(self.camera_logic)
        self.camera_logic().spectrum_acquired.connect(self.window._update_plot)
        self.show()

        
    def show(self):
        self.window.show()

    def on_deactivate(self):
        self.camera_logic().spectrum_acquired.disconnect(self.window._update_plot)
        self.window.close()