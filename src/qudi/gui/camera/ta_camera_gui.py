__all__ = ['CameraGuiModule'] 
from qudi.core.module import GuiBase
from qudi.util.signals import QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from qudi.core.connector import Connector
from matplotlib.figure import Figure
import numpy as np
from PyQt5 import QtWidgets
import os


class MainWindow(QtWidgets.QMainWindow):
    """ GUI module to continuously display TA and reference spectra. """



    def __init__(self,camera_logic):
        super().__init__()
        self.setWindowTitle("TA Camera Spectrum Viewer")
        self.logic = camera_logic()

        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QtWidgets.QVBoxLayout(self.central_widget)

        self.canvas = FigureCanvas(Figure(figsize=(6, 6)))
        layout.addWidget(self.canvas)

        self.ax1 = self.canvas.figure.add_subplot(211)
        self.ax1.set_xlabel("pixels")
        self.ax1.set_ylabel("TA Intensity")
        self.ta_line, = self.ax1.plot([], [], label='TA Spectrum')
        self.ax1.legend()

        self.ax2 = self.canvas.figure.add_subplot(212)
        self.ax2.set_xlabel("pixels")
        self.ax2.set_ylabel("Intensity")
        self.ref_line, = self.ax2.plot([], [], label='White Light')
        self.ax2.legend()

        controls_layout = QtWidgets.QHBoxLayout()

        self.start_button = QtWidgets.QPushButton("Start")
        self.start_button.clicked.connect(self._toggle_acquisition)
        controls_layout.addWidget(self.start_button)

        self.nframe_label = QtWidgets.QLabel("Frames:")
        controls_layout.addWidget(self.nframe_label)

        self.nframe_spin = QtWidgets.QSpinBox()
        self.nframe_spin.setMinimum(1)
        self.nframe_spin.setMaximum(10000)
        self.nframe_spin.setValue(self.logic.get_nframes())
        self.nframe_spin.valueChanged.connect(self._update_nframes)
        controls_layout.addWidget(self.nframe_spin)


        self.save_button = QtWidgets.QPushButton("Save Last Spectrum")
        self.save_button.clicked.connect(self._save_spectra)
        controls_layout.addWidget(self.save_button)

        layout.addLayout(controls_layout)

    def _toggle_acquisition(self):
        if self.start_button.text() == "Start":
            self.logic.start_acquisition()
            self.start_button.setText("Stop")
        else:
            self.logic.stop_acquisition()
            self.start_button.setText("Start")

    def _update_nframes(self):
        new_val = self.nframe_spin.value()
        self.logic.set_nframes(new_val)

    def _save_spectra(self):
        ta, reference = self.logic.get_last_spectra()
        if ta is None or reference is None:
            QtWidgets.QMessageBox.warning(self.main_window, "Warning", "No spectra acquired yet.")
            return

        path, _ = QtWidgets.QFileDialog.getSaveFileName(self.main_window, "Save Spectra", os.getcwd(), "CSV Files (*.csv)")
        if path:
            data = np.vstack([ta, reference]).T
            np.savetxt(path, data, delimiter=',', header='TA_Spectrum,Reference', comments='')
            QtWidgets.QMessageBox.information(self.main_window, "Saved", f"Spectra saved to {path}")

    def _update_plot(self, ta_spectrum, reference):
        x = np.arange(0, 2048)
        self.ta_line.set_data(x, ta_spectrum)
        self.ref_line.set_data(x, reference)
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.canvas.draw()

class CameraGuiModule(GuiBase):
    camera_logic = Connector(interface='TACameraLogic')

    def on_activate(self):
        self.camera_logic().spectrum_acquired.connect(self._update_plot)
        self.window = MainWindow(self.camera_logic)
        self.show()
        
    def show(self):
        self.window.show()

    def on_deactivate(self):
        self.logic.spectrum_acquired.disconnect(self._update_plot)
        self.window.close()