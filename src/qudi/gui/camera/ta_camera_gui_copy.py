__all__ = ['CameraGuiModule'] 
import sys
import os

from qudi.core.module import GuiBase
from qtpy import QtWidgets
import pyqtgraph as pg
from qudi.core.connector import Connector
import numpy as np



class MainWindow(QtWidgets.QMainWindow):
    """ GUI module to continuously display TA and reference spectra. """

    def __init__(self,camera_logic):
        super().__init__()
        self.logic = camera_logic

        # Create the main Qt windget 
        self.w = QtWidgets.QWidget()
        self.w.setWindowTitle('TA test')
        self.setCentralWidget(self.w)




        #Create TA spectrum display widget
        self.plot_widget_ta = pg.PlotWidget(title="TA Spectrum")
        self.plot_widget_ta.setLabel('left', 'Intensity')
        self.plot_widget_ta.setLabel('bottom', 'Pixel')
        self.ta_line = self.plot_widget_ta.plot([], pen='y')


        # White light reference display widget
        self.plot_widget_ref = pg.PlotWidget(title="White Light Reference")
        self.plot_widget_ref.setLabel('left', 'Intensity')
        self.plot_widget_ref.setLabel('bottom', 'Pixel')
        self.ref_line = self.plot_widget_ref.plot([], pen='c')

        # Start/stop button
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self._toggle_acquisition)
        self.start_button.setMinimumHeight(40)


        # Change the number of frame spinbox
        self.nframe_label = QtWidgets.QLabel("Frames:")
        self.nframe_spin = QtWidgets.QSpinBox()
        self.nframe_spin.setMinimum(1)
        self.nframe_spin.setMaximum(10000)
        self.nframe_spin.setValue(self.logic().get_nframes())
        self.nframe_spin.valueChanged.connect(self._update_nframes)
        self.nframe_spin.setMinimumHeight(40)


        #Create a grid layout to manage wigetts size abd position
        self.layout = QtWidgets.QGridLayout()
        self.w.setLayout(self.layout)

        # Add widgets to the layout
        self.layout.addWidget(self.plot_widget_ta,0,0) # upper left
        self.layout.addWidget(self.plot_widget_ref,0,1) # upper right
        self.layout.addWidget(self.start_button,1,0) # lower left
        self.layout.addWidget(self.nframe_label,1,1) # lower right
        self.layout.addWidget(self.nframe_spin,1,2)


    def _toggle_acquisition(self):
        if self.start_button.text() == "Start":
            self.logic().start_acquisition()
            self.start_button.setText("Stop")
        else:
            self.logic().stop_acquisition()
            self.start_button.setText("Start")

    def _update_nframes(self):
        new_val = self.nframe_spin.value()
        self.logic().set_nframes(new_val)

    def _update_plot(self, ta_spectrum, reference):
        x = np.arange(0, 2048)
        self.ta_line.setData(x, ta_spectrum)
        self.ref_line.setData(x, reference)

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