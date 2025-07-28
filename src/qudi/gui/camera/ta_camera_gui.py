from qudi.core.module import GuiBase
from qudi.util.signals import QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
from PyQt5 import QtWidgets
import os


class CameraGui(GuiBase):
    """ GUI module to continuously display TA and reference spectra. """

    _modclass = 'CameraGui'
    _modtype = 'gui'

    def on_activate(self):
        self.logic = self.get_module('TACameraLogic')
        self.logic.spectrum_acquired.connect(self._update_plot)
        self._build_ui()
        self.main_window.show()

    def _build_ui(self):
        self.main_window = QtWidgets.QMainWindow()
        self.main_window.setWindowTitle("TA Camera Spectrum Viewer")
        self.main_window.setMinimumSize(600, 450)

        central_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central_widget)
        self.main_window.setCentralWidget(central_widget)

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
