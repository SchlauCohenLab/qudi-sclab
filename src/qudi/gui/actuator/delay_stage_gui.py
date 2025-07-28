# -*- coding: utf-8 -*-

"""
DelayStageGui - GUI module for controlling a single-axis delay stage. Will not work if the stage has more than one axis

This GUI displays a horizontal slider that lets the user adjust the stage position
in millimeters, and sends the position (converted to meters) to the logic layer.

"""

__all__ = ['DelayStageGui'] 

from qudi.core.module import GuiBase
from qudi.core.connector import Connector
from qudi.core.configoption import ConfigOption
from PyQt5 import QtWidgets, QtCore


class MainWindow(QtWidgets.QMainWindow):
    """ Main GUI window containing a slider and label for delay stage control. """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Delay Stage Controller")
        self.setDockNestingEnabled(True)
        self.setMinimumSize(400, 100)

        # Central widget and layout
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Slider setup (position control in mm)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.slider.valueChanged.connect(self._slider_changed)

        # Label to show current position
        self.label = QtWidgets.QLabel("Delay: -- mm")
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.slider)

        # Default conversion factor (mm → m)
        self._conversion_factor = 1e-3

    def set_slider_range(self, min_m, max_m):
        """ Set slider range based on min/max in meters (converted to mm). """
        min_mm = int(min_m * 1e3)
        max_mm = int(max_m * 1e3)
        self._conversion_factor = 1e-3
        self.slider.setMinimum(min_mm)
        self.slider.setMaximum(max_mm)
        mid = (min_mm + max_mm) // 2
        self.slider.setValue(mid)
        self.label.setText(f"Delay: {mid} mm")

    def _slider_changed(self, value):
        """ Triggered when user moves slider; sends value to logic. """
        self.label.setText(f"Delay: {value} mm")
        if hasattr(self, "on_slider_moved"):
            delay_m = value * self._conversion_factor
            self.on_slider_moved(delay_m)

    def update_label_from_position(self, pos_m):
        """ Updates the GUI label based on external stage movement (in meters). """
        self.label.setText(f"Delay: {pos_m * 1e3:.3f} mm")


class DelayStageGui(GuiBase):
    """ GUI module for interacting with a delay stage via DelayStageLogic.
     
    Example config for copy-paste:

    delay_stage_gui::
        module.Class:  'actuator.delay_stage_gui.DelayStageGui'
        connect:
            delay_stage_logic: actuator_logic
    """
    # declare connectors
    actuator_logic = Connector(interface='DelayStageLogic')


    

    def on_activate(self):
        """ Initializes GUI, connects logic, sets up slider from hardware limits. """
        self.actuator_logic().sigUpdatePosition.connect(self._update_label)

        self.main_window = MainWindow()
        self.main_window.on_slider_moved = self._move_stage
        self.main_window.show()

        # Use hardware constraints to set slider limits
        axes = self.actuator_logic().get_constraints()
        self.main_window.set_slider_range(axes[0].step_range[0]/1e3, axes[0].step_range[1]/1e3)
             
    def on_deactivate(self):
        self.actuator_logic().sigUpdatePosition.disconnect(self._update_label)
        self.main_window.close()
    
    def show(self):
        self.main_window.show()

    def _move_stage(self, delay_m):
        """ Sends new stage position (in meters) to logic module. """
        self.actuator_logic().set_delay(delay_m)

    def _update_label(self, pos_dict):
        """ Receives updated stage position from logic and updates label. """
        if self._axis in pos_dict:
            self.main_window.update_label_from_position(pos_dict[self._axis])
