# -*- coding: utf-8 -*-

"""
DelayStageGui - GUI module for controlling a single-axis delay stage. Will not work if the stage has more than one axis


This GUI displays a horizontal slider that lets the user adjust the stage position
in millimeters, and sends the position (converted to meters) to the logic layer.

"""

__all__ = ['DelayStageGui'] 

from qudi.core.module import GuiBase
from qudi.core.connector import Connector
from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np

class DelayStageGui(GuiBase):
        """ GUI module for interacting with a delay stage via DelayStageLogic.
    
    Example config for copy-paste:

    delay_stage_gui::
        module.Class:  'actuator.delay_stage_gui.DelayStageGui'
        connect:
            actuator_logic: 'actuator_logic'
    """
    # declare connectors
    actuator_logic = Connector(interface='DelayStageLogic')

    def on_activate(self):
        self.axis =self.actuator_logic().get_constraints()[0]
        self.axis_name = self.axis.name
        self.actuator_logic().sigUpdatePosition.connect(self._update_label)


        self._moving = False
        self._last_position = None

        self.main_window = QtWidgets.QMainWindow()
        self.main_window.setWindowTitle("Delay Stage Controller")
        self.central_widget = QtWidgets.QWidget()
        self.main_window.setCentralWidget(self.central_widget)

        layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Position display
        self.position_display = QtWidgets.QLabel("Position: -- mm")
        self.position_display.setStyleSheet("font-size: 30px; padding: 20px; border: 2px solid black;")
        self.position_display.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.position_display)

        # Arrow control and step
        arrow_layout = QtWidgets.QHBoxLayout()
        self.down_button = QtWidgets.QPushButton("▼")
        self.up_button = QtWidgets.QPushButton("▲")
        self.step_input = QtWidgets.QLineEdit("0.1")
        self.step_input.setFixedWidth(60)
        self.step_input.setValidator(QtGui.QDoubleValidator())
        arrow_layout.addWidget(self.down_button)
        arrow_layout.addWidget(self.step_input)
        arrow_layout.addWidget(self.up_button)
        layout.addLayout(arrow_layout)

        # Min and Max
        min_max_layout = QtWidgets.QHBoxLayout()
        self.min_button = QtWidgets.QPushButton("Go Min")
        self.max_button = QtWidgets.QPushButton("Go Max")
        min_max_layout.addWidget(self.min_button)
        min_max_layout.addWidget(self.max_button)
        layout.addLayout(min_max_layout)

        # Goto position
        goto_layout = QtWidgets.QHBoxLayout()
        self.goto_input = QtWidgets.QLineEdit()
        self.goto_input.setPlaceholderText("Enter position [mm]")
        self.goto_button = QtWidgets.QPushButton("Go")
        goto_layout.addWidget(self.goto_input)
        goto_layout.addWidget(self.goto_button)
        layout.addLayout(goto_layout)

        # Movement lights
        light_layout = QtWidgets.QHBoxLayout()
        self.green_light = QtWidgets.QLabel()
        self.red_light = QtWidgets.QLabel()
        for light in (self.green_light, self.red_light):
            light.setFixedSize(20, 20)
            light.setStyleSheet("border-radius: 10px; background-color: gray;")
        light_layout.addWidget(QtWidgets.QLabel("Ready:"))
        light_layout.addWidget(self.green_light)
        light_layout.addStretch()
        light_layout.addWidget(QtWidgets.QLabel("Moving:"))
        light_layout.addWidget(self.red_light)
        layout.addLayout(light_layout)

        # Button press timers for continuous move
        self.up_timer = QtCore.QTimer()
        self.down_timer = QtCore.QTimer()
        self.up_timer.timeout.connect(lambda: self.move_stage(+1))
        self.down_timer.timeout.connect(lambda: self.move_stage(-1))

        self.up_button.pressed.connect(lambda: self.up_timer.start(100))
        self.up_button.released.connect(self.up_timer.stop)
        self.down_button.pressed.connect(lambda: self.down_timer.start(100))
        self.down_button.released.connect(self.down_timer.stop)

        # Connections
        self.goto_button.clicked.connect(self.goto_position)
        self.min_button.clicked.connect(lambda: self.actuator_logic.move_abs({self.axis_name: self.axis.step_range[0]/1e3}))
        self.max_button.clicked.connect(lambda: self.actuator_logic.move_abs({self.axis_name: self.axis.step_range[1]/1e3}))

        # Start polling timer to simulate moving light
        self.poll_timer = QtCore.QTimer()
        self.poll_timer.timeout.connect(self.poll_movement_status)
        self.poll_timer.start(200)

        self.main_window.show()
        
        def move_stage(self, direction):
            try:
                step = float(self.step_input.text())/1e3
                self.actuator_logic().set_delay_rel({self.axis_name: direction * step})
            except Exception as e:
                print("Invalid step or position:", e)

        def goto_position(self):
            try:
                value = float(self.goto_input.text())/1e3
                self.actuator_logic().set_delay_abs({self.axis_name: value})
            except Exception as e:
                print("Invalid position input:", e)

        def update_position(self):
            pos = self.actuator_logic().get_delay()
            self.position_display.setText(f"Position: {pos:.3f} mm")
            self._last_position = pos

        def poll_movement_status(self):
            current = self.actuator_logic().get_delay()
            if self._last_position is None:
                self._last_position = current

            if abs(current - self._last_position) > 1e-4:
                # Moving
                self._moving = True
                self.red_light.setStyleSheet("border-radius: 10px; background-color: red;")
                self.green_light.setStyleSheet("border-radius: 10px; background-color: gray;")
            else:
                # Idle
                self._moving = False
                self.red_light.setStyleSheet("border-radius: 10px; background-color: gray;")
                self.green_light.setStyleSheet("border-radius: 10px; background-color: green;")

            self._last_position = current
