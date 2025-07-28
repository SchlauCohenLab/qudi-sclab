# -*- coding: utf-8 -*-

__all__ = ['ActuatorLogic']

from PySide2 import QtCore

from qudi.core.module import LogicBase
from qudi.core.connector import Connector
from qudi.core.statusvariable import StatusVar
from qudi.core.configoption import ConfigOption
from qudi.util.mutex import RecursiveMutex

# qudi logic measurement modules must inherit qudi.core.module.LogicBase or other logic modules.
class DelayStageLogic(LogicBase):
    """ Simple logic module for interacting with the hardware actuators.

    actuator_logic:
        module.Class: 'delay_stage_logic.DelayStageLogic'
        connect:
            actuator: delay_stage
    """

    # connector for one actuator, if multiple actuators are needed use the ActuatorCombinerInterfuse
    actuator = Connector(interface='ActuatorInterface')

    _scan_min = StatusVar('scan_min', default={})
    _scan_max = StatusVar('scan_max', default={})
    _scan_step = StatusVar('scan_step', default={})

    sigUpdatePosition = QtCore.Signal(dict)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = RecursiveMutex()

    def on_activate(self):
        """ Activate module
        """
        self._axes = self.actuator().get_constraints()
        self._actual_position = self.actuator().get_pos()

        self.log.info('DelayStageLogic activated')
        
    def on_deactivate(self):
        """ Deactivate module
        """
        self._watchdog_active = False

    def set_delay(self, value):
        self.actuator().move_abs({'x1': value})
        pos = self.actuator().get_pos()
        return pos
    
    def get_delay(self):
        pos = self.actuator().get_pos()
        return pos

    def get_constraints(self):
        return self.actuator().get_constraints()

