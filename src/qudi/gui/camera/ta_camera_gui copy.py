__all__ = ['CameraGuiModule'] 
from qudi.core.module import GuiBase
from PyQt5 import QtWidgets
import pyqtgraph as pg
from qudi.core.connector import Connector
import numpy as np
import os


class MainWindow(QtWidgets.QMainWindow):
    """ GUI module to continuously display TA and reference spectra. """

    def __init__(self,camera_logic):
        super().__init__()
        self.setWindowTitle("TA Camera Spectrum Viewer")
        self.logic = camera_logic

        # Create the main Qt windget 
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QtWidgets.QVBoxLayout()
        self.central_widget.setLayout(layout)

        # TA spectrum display
        self.plot_widget_ta = pg.PlotWidget(title="TA Spectrum")
        self.plot_widget_ta.setLabel('left', 'Intensity')
        self.plot_widget_ta.setLabel('bottom', 'Pixel')
        self.ta_line = self.plot_widget_ta.plot([], pen='y', name="TA Spectrum")
        layout.addWidget(self.plot_widget_ta)


        # White light reference display
        self.plot_widget_ref = pg.PlotWidget(title="White Light Reference")
        self.plot_widget_ref.setLabel('left', 'Intensity')
        self.plot_widget_ref.setLabel('bottom', 'Pixel')
        self.ref_line = self.plot_widget_ref.plot([], pen='c', name="Reference")
        layout.addWidget(self.plot_widget_ref)

        #Controls layout
        controls_layout = QtWidgets.QHBoxLayout()

        # Start/stop button
        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self._toggle_acquisition)
        self.start_button.setMinimumHeight(40)
        controls_layout.addWidget(self.start_button)

        # Change the number of frame spinbox
        self.nframe_label = QtWidgets.QLabel("Frames:")
        controls_layout.addWidget(self.nframe_label)

        self.nframe_spin = QtWidgets.QSpinBox()
        self.nframe_spin.setMinimum(1)
        self.nframe_spin.setMaximum(10000)
        self.nframe_spin.setValue(self.logic().get_nframes())
        self.nframe_spin.valueChanged.connect(self._update_nframes)
        self.nframe_spin.setMinimumHeight(40)
        controls_layout.addWidget(self.nframe_spin)

        # Save the aquired spectrum
        self.save_button = QtWidgets.QPushButton("Save Spectrum")
        self.save_button.clicked.connect(self._save_spectra)
        self.save_button.setMinimumHeight(40)
        controls_layout.addWidget(self.save_button)

        layout.addLayout(controls_layout)

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

    def _save_spectra(self):
        ta, reference = self.logic().get_last_spectra()
        if ta is None or reference is None:
            QtWidgets.QMessageBox.warning(self, "Warning", "No spectra acquired yet.")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Spectra", os.getcwd(), "CSV Files (*.csv)")
        if path:
            data = np.vstack([ta, reference]).T
            np.savetxt(path, data, delimiter=',', header='TA_Spectrum,Reference', comments='')
            QtWidgets.QMessageBox.information(self, "Saved", f"Spectra saved to {path}")
    
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