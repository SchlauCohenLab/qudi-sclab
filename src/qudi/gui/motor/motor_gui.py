# -*- coding: utf-8 -*-

"""
This file contains the qudi time series streaming gui.

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

__all__ = ['MotorGui']

from enum import IntEnum
from PySide2 import QtWidgets, QtCore, QtGui
import os
from functools import partial

import qudi.util.uic as uic
from qudi.core.connector import Connector
from qudi.util.units import ScaledFloat
from qudi.core.statusvariable import StatusVar
from qudi.core.module import GuiBase
from qudi.core.configoption import ConfigOption
from qudi.util.widgets.scientific_spinbox import ScienDSpinBox

from qudi.gui.motor.axis_control_dockwidget import AxisDockWidget


class MainWindow(QtWidgets.QMainWindow):
    """Main Window for the SwitchGui module"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        #self.setWindowTitle('qudi: <INSERT HARDWARE NAME>')
        # Create main layout and central widget
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        self.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.setDockNestingEnabled(True)
        self.setMinimumSize(300, 200)
        return

class MotorGui(GuiBase):
    """
    A graphical interface to control the actuator displacement.

    Example config for copy-paste:

    motor_gui:
        module.Class: 'actuator.motor_gui.MotorGui'
        connect:
            motor_logic: 'motor_logic'

    """

    # declare connectors
    motor_logic = Connector(interface='MotorLogic')

    # declare status variables
    _axes_displacement = ConfigOption(name='axes_displacement', default={})
    _abs_displacement = ConfigOption(name='abs_displacement', default={})

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mw = None
        self._widgets = dict()

    def on_activate(self):
        """ Create all UI objects and show the window.
        """
        self._mw = MainWindow()
        self._constraints = self.motor_logic().constraints

        self.axes_widgets = {}
        for axis in self.motor_logic().axes:
            axis_widget = AxisDockWidget(axis)
            axis_widget.setAllowedAreas(QtCore.Qt.LeftDockWidgetArea)
            self.axes_widgets[axis] = axis_widget
            self._mw.addDockWidget(QtCore.Qt.LeftDockWidgetArea, axis_widget)
            self.init_axis_widgets(axis)

        wg_list = [wg for wg in self.axes_widgets.values()]
        for i in range(len(wg_list)-1):
            self._mw.tabifyDockWidget(wg_list[i], wg_list[i+1])

        self.show()

    def show(self):
        """
        Show the GUI
        """
        self._mw.show()

    def on_deactivate(self):
        """ Hide window empty the GUI and disconnect signals
        """
        self._mw.close()

    def init_axis_widgets(self, axis):

        self.axes_widgets[axis].axis_label.setText("{}-Axis".format(axis.upper()))

        if axis not in self._abs_displacement.keys():
            self._abs_displacement[axis] = True
        self._update_radio_btn(axis, self._abs_displacement[axis])

        if axis not in self._axes_displacement.keys():
            self._axes_displacement[axis] = 0
        self.axes_widgets[axis].displacement_sb.setValue(self._axes_displacement[axis])
        self.axes_widgets[axis].displacement_sb.setSuffix(self._constraints[axis]['unit'])

        self.axes_widgets[axis].position_value.setText("{:.2r}{}".format(ScaledFloat(self.motor_logic().get_position([axis])[axis]),
                                                                     self._constraints[axis]['unit']))

        self.axes_widgets[axis].abs_displacement.clicked.connect(lambda: self._update_radio_btn(axis, True))
        self.axes_widgets[axis].rel_displacement.clicked.connect(lambda: self._update_radio_btn(axis, False))
        self.axes_widgets[axis].displacement_sb.editingFinished.connect(lambda: self._update_displacement_value(axis))
        self.axes_widgets[axis].home_btn.clicked.connect(lambda: self._home_axis(axis))
        self.axes_widgets[axis].move_btn.clicked.connect(lambda: self._move_axis(axis))

        self._update_status_timer = QtCore.QTimer()
        self._update_status_timer.setSingleShot(False)
        self._update_status_timer.timeout.connect(self._update_axis_status, QtCore.Qt.QueuedConnection)
        self._update_status_timer.start(100)

        self._update_position_timer = QtCore.QTimer()
        self._update_position_timer.setSingleShot(False)
        self._update_position_timer.timeout.connect(self._update_position_value, QtCore.Qt.QueuedConnection)
        self._update_position_timer.start(10)

        #self.axes_widgets[axis].scan_btn.clicked.connect(lambda: self.motor_logic().start_scan(axis))

    def _update_radio_btn(self, axis, absolute):

        position = self.motor_logic().get_position([axis])[axis]
        if absolute:
            self.axes_widgets[axis].rel_displacement.setChecked(False)
            self.axes_widgets[axis].abs_displacement.setChecked(True)
            self._abs_displacement[axis] = True
            self.axes_widgets[axis].displacement_sb.setRange(self._constraints[axis]['pos_min'],
                                                             self._constraints[axis]['pos_max'])
        else:
            self.axes_widgets[axis].rel_displacement.setChecked(True)
            self.axes_widgets[axis].abs_displacement.setChecked(False)
            self._abs_displacement[axis] = False

            self.axes_widgets[axis].displacement_sb.setRange(self._constraints[axis]['pos_min'] - position,
                                                             self._constraints[axis]['pos_max'] - position)

    def _update_axis_status(self):

        axes_status = self.motor_logic().status
        for axis, status in axes_status.items():
            if status == 0:
                self.axes_widgets[axis].axis_status.setText("Idle")
            if status == -1:
                self.axes_widgets[axis].axis_status.setText("Error")

    def _update_position_value(self):

        axes_position = self.motor_logic().position
        for axis, pos in axes_position.items():
            self.axes_widgets[axis].position_value.setText("{:.2r}{}".format(ScaledFloat(pos), self._constraints[axis]['unit']))

    def _update_displacement_value(self, axis):

        self._axes_displacement[axis] = self.axes_widgets[axis].displacement_sb.value()

    def _home_axis(self, axis):

        self.motor_logic().home([axis])

    def _move_axis(self, axis):

        displacement = self.axes_widgets[axis].displacement_sb.value()
        self._axes_displacement[axis] = displacement
        if self._abs_displacement[axis]:
            self.motor_logic().move_abs({axis: displacement})
        else:
            self.motor_logic().move_rel({axis: displacement})