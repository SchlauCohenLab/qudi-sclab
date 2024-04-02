# -*- coding: utf-8 -*-

__all__ = ['PumpProbeLogic']

from PySide2 import QtCore
from collections import OrderedDict
import datetime
import matplotlib.pyplot as plt
import numpy as np
import time

from qudi.core.module import LogicBase
from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex

class PumpProbeLogic(LogicBase):

    delay_stage = Connector(name="delay_stage", interface="MotorInterface", optional=False)
    counter = Connector(name="counter", interface="FastCounterInterface", optional=False)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._mutex = Mutex()  # Mutex for access serialization

    def on_activate(self):
        pass

    def on_deactivate(self):
        pass

    @property
    def delay_range(self):
        pass