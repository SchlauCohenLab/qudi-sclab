# -*- coding: utf-8 -*-

"""
Combine two hardware switches into one.

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
import numpy as np
import time

from PySide2 import QtCore

from qudi.interface.scanner_interface import ScannerInterface, ScanData, ScanConstraints
from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.util.mutex import RecursiveMutex

class ScannerInterfuse(ScannerInterface):
    """ Methods to control slow (mechanical) laser switching devices.
    This interfuse in particular combines two switches into one.

    Example config for copy-paste:

    scanner_interfuse:
        module.Class: 'interfuse.scanner_interfuse.ScannerInterfuse'
        connect:
            detector: 'detector'
            actuator: 'actuator'

    """

    # connectors for the switches to be combined
    _detector = Connector(name='detector', interface='DetectorInterface')
    _actuator = Connector(name='actuator', interface='ActuatorInterface')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._line_pointer = 0

        self._current_scan_frequency = -1
        self._current_scan_ranges = [tuple(), tuple()]
        self._current_scan_axes = tuple()
        self._current_scan_resolution = tuple()

        self._scan_data = None
        self._scan_dict = None
        self.raw_data_container = None

        self._constraints = None

        self._target_pos = dict()
        self._stored_target_pos = dict()
        self._start_scan_after_cursor = False
        self._abort_cursor_move = False

        self._scan_point_timer = None
        self._min_step_interval = 1e-3
        self._scanner_distance_atol = 1e-9

        self._thread_lock_cursor = RecursiveMutex()
        self._thread_lock_data = RecursiveMutex()

        self._2D_scan_sig = QtCore.Signal()

    def on_activate(self):
        """ Activate the module and fill status variables.
        """
        self._scan_channels = self._detector().get_constraints()
        self._scan_axes = self._actuator().get_constraints()
        self._constraints = ScanConstraints(axes=self._scan_axes,
                                            channels=self._scan_channels,
                                            backscan_configurable=False,  # TODO incorporate in scanning_probe toolchain
                                            has_position_feedback=False,  # TODO incorporate in scanning_probe toolchain
                                            square_px_only=False)

        self._target_pos = self._actuator().get_pos()

        self.scan_point_function = None
        self.scan_point_channels = None

        self._start_scanning_timer = QtCore.QTimer(parent=self)

        self._start_scanning_timer.setSingleShot(True)
        self._start_scanning_timer.timeout.connect(self._wait_before_scanning, QtCore.Qt.QueuedConnection)
        self._start_scanning_timer.setInterval(1)  # (ms), dynamically calculated during write loop

    def on_deactivate(self):
        """ Deactivate the module and clean up.
        """
        self._actuator().abort()
        self._detector().stop_measure()


    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        return self._constraints

    def reset(self):
        """ Hard reset of the hardware.
        """
        if self._actuator().module_state() == 'locked':
            self._actuator().abort()

        if self._actuator().module_state() != 'locked':
            self._actuator().home()

        if self._detector().module_state() == 'locked':
            self._detector().stop_measure()

    def configure_scan(self, scan_settings):
        """ Configure the hardware with all parameters needed for a 1D or 2D scan.

        @param ScanSettings settings: ScanSettings instance holding all parameters # TODO update me!

        @return (bool, ScanSettings): Failure indicator (fail=True),
                                      altered ScanSettings instance (same as "settings")
        """
        """ Configure the hardware with all parameters needed for a 1D or 2D scan.

                @param dict scan_settings: scan_settings dictionary holding all the parameters 'axes', 'resolution', 'ranges'
                #  TODO update docstring in interface

                @return (bool, ScanSettings): Failure indicator (fail=True),
                                              altered ScanSettings instance (same as "settings")
                """

        if self.module_state()=='locked':
            self.log.error('Unable to configure scan parameters while scan is running. '
                           'Stop scanning and try again.')
            return True, self.scan_settings

        axes = scan_settings.get('axes', self._current_scan_axes)
        ranges = tuple(
            (min(r), max(r)) for r in scan_settings.get('range', self._current_scan_ranges)
        )
        resolution = scan_settings.get('resolution', self._current_scan_resolution)
        frequency = float(scan_settings.get('frequency', self._current_scan_frequency))

        if not set(axes).issubset(self._position_ranges):
            self.log.error('Unknown axes names encountered. Valid axes are: {0}'
                           ''.format(set(self._position_ranges)))
            return True, self.scan_settings

        if len(axes) != len(ranges) or len(axes) != len(resolution):
            self.log.error('"axes", "range" and "resolution" must have same length.')
            return True, self.scan_settings
        for i, ax in enumerate(axes):
            for axis_constr in self._constraints.axes.values():
                if ax == axis_constr.name:
                    break
            if ranges[i][0] < axis_constr.min_value or ranges[i][1] > axis_constr.max_value:
                self.log.error('Scan range out of bounds for axis "{0}". Maximum possible range'
                               ' is: {1}'.format(ax, axis_constr.value_range))
                return True, self.scan_settings
            if resolution[i] < axis_constr.min_resolution or resolution[i] > axis_constr.max_resolution:
                self.log.error('Scan resolution out of bounds for axis "{0}". Maximum possible '
                               'range is: {1}'.format(ax, axis_constr.resolution_range))
                return True, self.scan_settings
            if i == 0:
                if frequency < axis_constr.min_frequency or frequency > axis_constr.max_frequency:
                    self.log.error('Scan frequency out of bounds for fast axis "{0}". Maximum '
                                   'possible range is: {1}'
                                   ''.format(ax, axis_constr.frequency_range))
                    return True, self.scan_settings
            with self._thread_lock_data:
                try:
                    self._scan_data = ScanData(
                        channels=tuple(self._constraints.channels.values()),
                        scan_axes=tuple(self._constraints.axes[ax] for ax in axes),
                        scan_range=ranges,
                        scan_resolution=tuple(resolution),
                        scan_frequency=frequency,
                        position_feedback_axes=None
                    )
                    self.raw_data_container = RawDataContainer(self._scan_data.channels,
                                                               resolution[
                                                                   1] if self._scan_data.scan_dimension == 2 else 1,
                                                               resolution[0],
                                                               self.__backwards_line_resolution)

                    self._scan_dict = self.initialize_scan_arrays(self._scan_data)

                    # self.log.debug(f"New scanData created: {self._scan_data.data}")

                except:
                    self.log.exception("")
                    return True, self.scan_settings

            self._current_scan_resolution = tuple(resolution)
            self._current_scan_ranges = ranges
            self._current_scan_axes = tuple(axes)
            self._current_scan_frequency = frequency

            return False, self.scan_settings

    def move_absolute(self, position, velocity=None, blocking=False):
        """ Move the scanning probe to an absolute position as fast as possible or with a defined
        velocity.

        Log error and return current target position if something fails or a scan is in progress.

        @param bool blocking: If True this call returns only after the final position is reached.
        """
        if self.module_state()=='locked':
            self.log.error('Cannot move the scanner while, scan is running')
            return self._target_pos

        self._target_pos = position
        self._actuator().move_abs(position)
        return self._target_pos

    def move_relative(self, distance, velocity=None, blocking=False):
        """ Move the scanning probe by a relative distance from the current target position as fast
        as possible or with a defined velocity.

        Log error and return current target position if something fails or a 1D/2D scan is in
        progress.

        @param bool blocking: If True this call returns only after the final position is reached.

        """
        if self.module_state() == 'locked':
            self.log.error('Cannot move the scanner while, scan is running')
            return self._target_pos

        self._target_pos = distance
        self._actuator().move_rel(distance)
        return self._target_pos

    def get_target(self):
        """ Get the current target position of the scanner hardware
        (i.e. the "theoretical" position).

        @return dict: current target position per axis.
        """
        return self._target_pos

    def get_position(self):
        """ Get a snapshot of the actual scanner position (i.e. from position feedback sensors).
        For the same target this value can fluctuate according to the scanners positioning accuracy.

        For scanning devices that do not have position feedback sensors, simply return the target
        position (see also: ScanningProbeInterface.get_target).

        @return dict: current position per axis.
        """
        return self._actuator().get_pos()

    def start_scan(self):

        try:

            if self.thread() is not QtCore.QThread.currentThread():
                QtCore.QMetaObject.invokeMethod(self, '_start_scan', QtCore.Qt.BlockingQueuedConnection)

            if self._scan_data is None:
                self.log.error('Scan Data is None. Scan settings need to be configured before starting')
                return -1

            if self.module_state() == 'locked':
                self.log.error('Cannot start a scan while scanning probe is already running')
                return -1

            with self._thread_lock_data:
                self._scan_data.new_scan()
                #self.log.debug(f"New scan data: {self._scan_data.data}, position {self._scan_data._position_data}")
                self._stored_target_pos = self.get_target().copy()
                self._scan_data.scanner_target_at_start = self._stored_target_pos

            self.module_state.lock()

            self._line_pointer = 0

            first_scan_position = {ax: pos[0] for ax, pos
                                   in zip(self.scan_settings['axes'], self.scan_settings['range'])}


            self._actuator().move_abs(first_scan_position)

            self._start_scanning_timer.start()

        except Exception:
            self.module_state.unlock()
            self.log.error("Scan failed cyka blyat ! You have to do much better !")
            return -1

        return 0

    def _wait_before_scanning(self):

        current_pos_vec = self._pos_dict_to_vec(self.get_position())

        target_pos_vec = self._pos_dict_to_vec(self._target_pos)
        connecting_vec = target_pos_vec - current_pos_vec
        distance_to_target = np.linalg.norm(connecting_vec)

        # Terminate follow loop if target is reached
        if distance_to_target > self._scanner_distance_atol:
            self._start_scanning_timer.start()
            return
        else:
            self._start_scanning_timer.stop()
            self._start_scanning()

    def _start_scanning(self):

        if self._scan_data.scan_dimension == 1:

            axis = self._scan_axes[0]
            horizontal_resolution = self._scan_data.scan_resolution[0]

            forward_line_path = np.linspace(*self._scan_data.scan_range[0], horizontal_resolution)
            backward_line_path = np.linspace(self._scan_data.scan_range[1], self._scan_data.scan_range[0], self.__backwards_line_resolution)

            line_path = np.concatenate((forward_line_path, backward_line_path))

            self._scan_line(self, line_path, axis)

        elif self._scan_data.scan_dimension == 2:

            horizontal_resolution = self._scan_data.scan_resolution[0]
            vertical_resolution = self._scan_data.scan_resolution[1]

            # horizontal scan array / "fast axis"
            haxis = self._scan_data.scan_axes[0]
            vaxis = self._scan_data.scan_axes[1]

            forward_hline_path = np.linspace(*self._scan_data.scan_range[0], horizontal_resolution)
            backward_hline_path = np.linspace(self._scan_data.scan_range[1], self._scan_data.scan_range[0], self.__backwards_line_resolution)
            hline_path = np.concatenate((forward_hline_path, backward_hline_path))
            vline_path = np.linspace(*self._scan_data.scan_range[1], vertical_resolution)

            self._2D_scan_sig.connect(lambda : self._2D_scan(self, hline_path, vline_path, haxis, vaxis))

            self._2D_scan_sig.emit()

    def _2D_scan(self, hline_path, vline_path, haxis, vaxis):

        position = vline_path[self._pointer]
        self._actuator().move_abs({vaxis: position})
        self._scan_line(hline_path, haxis)

    def _scan_line(self, line_path, axis):
        """ Scans a line and returns the counts on that line.

        @param float[][4] line_path: array of 4-part tuples defining the voltage points
        @param bool pixel_clock: whether we need to output a pixel clock for this line

        @return float[]: the photon counts per second
        """
        line_path = line_path.T
        self._count_data = np.zeros((len(line_path), len(self.get_scanner_count_channels())))
        for i, position in enumerate(line_path):
            self._actuator().move_abs({axis : position})
            if self.scan_point_function:
                data = self.scan_point_function(point_index=i, point_position=position)
            else:
                data = self._detector().get_data()

            data_dict = {ch.name:data[i].flatten() for i, ch in enumerate(self._scan_channels)}

        with self._thread_lock_data:
            self.raw_data_container.fill_container(data_dict)
            self._scan_data.data = self.raw_data_container.forwards_data()

        if self._scan_data.scan_dimension == 2:
            self._pointer += 1
            self._2D_scan_sig.emit()

    def stop_scan(self):
        """

        @return bool: Failure indicator (fail=True)
        """
        self._start_scan_after_cursor = False  # Ensure Scan HW is not started after movement
        if self._ao_setpoint_channels_active:
            self._abort_cursor_movement()
            # self.log.debug("Move aborted")

        if self._actuator().is_running:
            self._actuator().abort()
            # self.log.debug("Frame stopped")

        self.module_state.unlock()
        # self.log.debug("Module unlocked")

        self.move_absolute(self._stored_target_pos)
        self._stored_target_pos = dict()

    def get_scan_data(self):
        """

        @return (bool, ScanData): Failure indicator (fail=True), ScanData instance used in the scan
        """
        if self._scan_data is None:
            raise RuntimeError('ScanData is not yet configured, please call "configure_scan" first')
        try:
            with self._thread_lock_data:
                return self._scan_data.copy()
        except:
            self.log.exception("")

    def emergency_stop(self):
        """

        @return:
        """
        self._actuator().abort()
        return False

    @property
    def is_move_running(self):
        with self._thread_lock_cursor:
            running = self.__t_last_follow is not None
            return running
    
    @property
    def scan_settings(self):

        settings = {'axes': tuple(self._current_scan_axes),
                    'range': tuple(self._current_scan_ranges),
                    'resolution': tuple(self._current_scan_resolution),
                    'frequency': self._current_scan_frequency}
        return settings

    def _pos_dict_to_vec(self, position):

        pos_list = [el[1] for el in sorted(position.items())]
        return np.asarray(pos_list)

    def set_advanced_acquisition(self, scan_point_function=None, scan_point_channels=None):
        """ Set the function to use at each point of the scan and the function giving the related channels name.

        @param (function) scan_point_function : function taking in argument the scanning index and position and returning
         a ndarray of acquired data.
        @param (function) scan_point_channels : function returning a list of channels with the same length than the array
        returned by scan_point_function.

        """

        self.scan_point_function = scan_point_function
        self.scan_point_channels = scan_point_channels

class RawDataContainer:

    def __init__(self, channel_keys, number_of_scan_lines, forward_line_resolution, backwards_line_resolution):
        self.forward_line_resolution = forward_line_resolution
        self.number_of_scan_lines = number_of_scan_lines
        self.forward_line_resolution = forward_line_resolution
        self.backwards_line_resolution = backwards_line_resolution

        self.frame_size = number_of_scan_lines * (forward_line_resolution + backwards_line_resolution)
        self._raw = {key: np.full(self.frame_size, np.nan) for key in channel_keys}

    def fill_container(self, samples_dict):
        # get index of first nan from one element of dict
        first_nan_idx = self.number_of_non_nan_values
        for key, samples in samples_dict.items():
            self._raw[key][first_nan_idx:first_nan_idx + len(samples)] = samples

    def forwards_data(self):
        reshaped_2d_dict = dict.fromkeys(self._raw)
        for key in self._raw:
            if self.number_of_scan_lines > 1:
                reshaped_arr = self._raw[key].reshape(self.number_of_scan_lines,
                                                      self.forward_line_resolution + self.backwards_line_resolution)
                reshaped_2d_dict[key] = reshaped_arr[:, :self.forward_line_resolution].T
            elif self.number_of_scan_lines == 1:
                reshaped_2d_dict[key] = self._raw[key][:self.forward_line_resolution]
        return reshaped_2d_dict

    def backwards_data(self):
        reshaped_2d_dict = dict.fromkeys(self._raw)
        for key in self._raw:
            if self.number_of_scan_lines > 1:
                reshaped_arr = self._raw[key].reshape(self.number_of_scan_lines,
                                                      self.forward_line_resolution + self.backwards_line_resolution)
                reshaped_2d_dict[key] = reshaped_arr[:, self.forward_line_resolution:].T
            elif self.number_of_scan_lines == 1:
                reshaped_2d_dict[key] = self._raw[key][self.forward_line_resolution:]

        return reshaped_2d_dict

    @property
    def number_of_non_nan_values(self):
        """
        returns number of not NaN samples
        """
        return np.sum(~np.isnan(next(iter(self._raw.values()))))

    @property
    def is_full(self):
        return self.number_of_non_nan_values == self.frame_size