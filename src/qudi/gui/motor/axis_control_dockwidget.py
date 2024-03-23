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

__all__ = ('AxisDockWidget', 'AxisWidget')

from PySide2 import QtWidgets, QtCore, QtGui
import os

import qudi.util.uic as uic
from qudi.util.widgets.scientific_spinbox import ScienDSpinBox


class AxisDockWidget(QtWidgets.QDockWidget):
    """ Scanner control QDockWidget based on the corresponding QWidget subclass
    """
    __wrapped_attributes = frozenset({'layout', 'axis_label',
                                      'axis_status', 'displacement_sb', 'position_value', 'abs_displacement',
                                      'rel_displacement', 'home_btn', 'move_btn', 'scan_btn', 'range_max', 'range_min',
                                      'range_res'})

    def __init__(self, axis, *args, **kwargs):
        super().__init__("{} axis".format(axis))
        self.setObjectName('axis_control_dockWidget')
        widget = AxisWidget()
        widget.setObjectName('axis_control_widget')
        self.setWidget(widget)
        self.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.setMinimumSize(300, 200)
        return

    def __getattr__(self, item):
        if item in self.__wrapped_attributes:
            return getattr(self.widget(), item)
        raise AttributeError('AxisControlDockWidget has not attribute "{0}"'.format(item))


class AxisWidget(QtWidgets.QWidget):
    """ Widget to control scan parameters and target position of scanner axes.
    """

    #sigResolutionChanged = QtCore.Signal(str, int)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.layout = QtWidgets.QGridLayout()

        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(20)
        self.layout = QtWidgets.QGridLayout()
        self.axis_label = QtWidgets.QLabel('<axis label>')
        self.axis_label.setFont(font)
        self.axis_label.setAlignment(QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.axis_label, 0, 0)

        font = QtGui.QFont()
        font.setBold(True)
        self.axis_status = QtWidgets.QLabel('<axis status>')
        self.axis_status.setFont(font)
        self.axis_status.setAlignment(QtCore.Qt.AlignRight)
        self.layout.addWidget(self.axis_status, 0, 1)

        self.displacement_sb = ScienDSpinBox()
        self.displacement_sb.setObjectName('<axis displacement>')
        self.displacement_sb.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.displacement_sb.setMinimumSize(100, 34)
        self.layout.addWidget(self.displacement_sb, 1, 0)

        font = QtGui.QFont()
        font.setBold(True)
        font.setPointSize(30)
        self.position_value = QtWidgets.QLabel('<axis position>')
        self.position_value.setFont(font)
        self.position_value.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.position_value, 1, 1)

        self.rb_layout = QtWidgets.QHBoxLayout()
        self.abs_displacement = QtWidgets.QRadioButton('Absolute')
        self.abs_displacement.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.abs_displacement.setMinimumSize(80, 34)
        self.rb_layout.addWidget(self.abs_displacement, 0)
        self.rel_displacement = QtWidgets.QRadioButton('Relative')
        self.rel_displacement.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.rel_displacement.setMinimumSize(80, 34)
        self.rb_layout.addWidget(self.rel_displacement, 1)
        self.layout.addLayout(self.rb_layout, 2, 0)

        self.btn_layout = QtWidgets.QHBoxLayout()
        self.home_btn = QtWidgets.QPushButton('Home')
        self.home_btn.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.btn_layout.addWidget(self.home_btn, 0)
        self.home_btn.setMinimumSize(80, 34)
        self.move_btn = QtWidgets.QPushButton('Move')
        self.move_btn.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.btn_layout.addWidget(self.move_btn, 1)
        self.move_btn.setMinimumSize(80, 34)
        self.layout.addLayout(self.btn_layout, 2, 1)

        self.hline = QtWidgets.QFrame()
        self.hline.setFrameShape(QtWidgets.QFrame.HLine)
        self.hline.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.layout.addWidget(self.hline, 3, 0, 1, 2)

        self.range_label_layout = QtWidgets.QHBoxLayout()
        self.min_label = QtWidgets.QLabel('Min')
        self.min_label.setMinimumSize(80, 10)
        self.min_label.setMargin(0)
        self.min_label.setAlignment(QtCore.Qt.AlignCenter)
        self.range_label_layout.addWidget(self.min_label, 0)
        self.max_label = QtWidgets.QLabel('Max')
        self.max_label.setMinimumSize(80, 10)
        self.max_label.setMargin(0)
        self.max_label.setAlignment(QtCore.Qt.AlignCenter)
        self.range_label_layout.addWidget(self.max_label, 1)
        self.step_label = QtWidgets.QLabel('Step')
        self.step_label.setMinimumSize(80, 10)
        self.step_label.setMargin(0)
        self.step_label.setAlignment(QtCore.Qt.AlignCenter)
        self.range_label_layout.addWidget(self.step_label, 2)
        self._blank_label = QtWidgets.QLabel('')
        self._blank_label.setMinimumSize(80, 10)
        self.range_label_layout.addWidget(self._blank_label, 3)
        self.range_label_layout.setAlignment(QtCore.Qt.AlignCenter)
        self.range_label_layout.setMargin(0)
        self.layout.addLayout(self.range_label_layout, 4, 0, 1, 2)

        self.range_layout = QtWidgets.QHBoxLayout()

        self.range_min = ScienDSpinBox()
        self.range_min.setObjectName('<axis range min>')
        self.range_min.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.range_min.setMinimumSize(80, 34)
        self.range_layout.addWidget(self.range_min, 0)

        self.range_max = ScienDSpinBox()
        self.range_max.setObjectName('<axis range max>')
        self.range_max.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.range_max.setMinimumSize(80, 34)
        self.range_layout.addWidget(self.range_max, 1)

        self.range_res = ScienDSpinBox()
        self.range_res.setObjectName('<axis range resolution>')
        self.range_res.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.range_res.setMinimumSize(80, 34)
        self.range_layout.addWidget(self.range_res, 2)

        self.scan_btn = QtWidgets.QPushButton('Scan')
        self.scan_btn.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.scan_btn.setMinimumSize(80, 34)
        self.range_layout.addWidget(self.scan_btn, 3)

        self.layout.addLayout(self.range_layout, 5, 0, 1, 2)

        self.setLayout(self.layout)
        self.setMaximumHeight(self.sizeHint().height())
