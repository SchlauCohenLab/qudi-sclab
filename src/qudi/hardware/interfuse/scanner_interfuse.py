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
        options:
            backwards_line_resolution: 3

    """

    # connectors for the switches to be combined
    _detector = Connector(name='detector', interface='DetectorInterface')
    _actuator = Connector(name='actuator', interface='ActuatorInterface')

    __backwards_line_resolution = ConfigOption(name='backwards_line_resolution', default=50)

    _sigStartScanning = QtCore.Signal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._pointer = 0

        self._current_scan_frequency = -1
        self._current_scan_ranges = [tuple(), tuple()]
        self._current_scan_axes = tuple()
        self._current_scan_resolution = tuple()

        self._scan_data = None
        self.raw_data_container = None

        self._constraints = None

        self._target_pos = dict()
        self._stored_target_pos = dict()

        self._scan_point_timer = None
        self._min_step_interval = 1e-3
        self._scanner_distance_atol = 100e-9

        self._thread_lock = RecursiveMutex()

        self._advanced_scan_channels = None
        self._advanced_scan_function = None
        self._advanced_scan = False

    def on_activate(self):
        """ Activate the module and fill status variables.
        """
        self._scan_channels = self._detector().get_constraints()
        self._scan_axes = self._actuator().get_constraints()

        self._position_ranges = {ax.name:ax.value_range for ax in self._scan_axes}
        self._constraints = ScanConstraints(axes=self._scan_axes,
                                            channels=self._scan_channels,
                                            backscan_configurable=False,  # TODO incorporate in scanning_probe toolchain
                                            has_position_feedback=False,  # TODO incorporate in scanning_probe toolchain
                                            square_px_only=False)

        self._target_pos = self._actuator().get_pos()

        self._sigStartScanning.connect(self._scan_point, QtCore.Qt.QueuedConnection)
        # self._start_scanning_timer = QtCore.QTimer()
        #
        # self._start_scanning_timer.setSingleShot(False)
        # self._start_scanning_timer.timeout.connect(self._scan_point, QtCore.Qt.QueuedConnection)

    def on_deactivate(self):
        """ Deactivate the module and clean up.
        """
        # self._start_scanning_timer.stop()
        # self._start_scanning_timer.timeout.disconnect()
        self._sigStartScanning.disconnect()
        if self.module_state() != 'idle':
            self.stop_scan()
        return

    def get_constraints(self):
        """ Get hardware constraints/limitations.

        @return dict: scanner constraints
        """
        with self._thread_lock:
            return self._constraints

    def reset(self):
        """ Hard reset of the hardware.
        """
        with self._thread_lock:
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
        with self._thread_lock:
            if self.module_state() == 'locked':
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

                try:
                    if self._advanced_scan:
                        channels = tuple(self._advanced_scan_channels)
                    else:
                        channels = tuple(self._constraints.channels.values())
                    self._scan_data = ScanData(
                        channels=channels,
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
                    # self.log.debug(f"New scanData created: {self._scan_data.data}")

                except:
                    self.log.exception("")
                    return True, self.scan_settings

                self._current_scan_resolution = tuple(resolution)
                self._current_scan_ranges = ranges
                self._current_scan_axes = tuple(axes)
                self._current_scan_frequency = frequency
                self._update_scanning_path()

                return False, self.scan_settings

    def move_absolute(self, position, velocity=None, blocking=False):
        """ Move the scanning probe to an absolute position as fast as possible or with a defined
        velocity.

        Log error and return current target position if something fails or a scan is in progress.

        @param bool blocking: If True this call returns only after the final position is reached.
        """
        with self._thread_lock:
            if self.module_state()=='locked':
                self.log.error('Cannot move the scanner while, scan is running')
                return self._target_pos

            for axis in self._scan_axes:
                if axis.name in position.keys():
                    self._target_pos[axis.name] = position[axis.name]

            self._actuator().move_abs(position)
            return self._target_pos

    def move_relative(self, distance, velocity=None, blocking=False):
        """ Move the scanning probe by a relative distance from the current target position as fast
        as possible or with a defined velocity.

        Log error and return current target position if something fails or a 1D/2D scan is in
        progress.

        @param bool blocking: If True this call returns only after the final position is reached.

        """
        with self._thread_lock:
            if self.module_state() == 'locked':
                self.log.error('Cannot move the scanner while, scan is running')
                return self._target_pos

            for axis in self._scan_axes:
                if axis.name in distance.keys():
                    self._target_pos[axis.name] = distance[axis.name]

            self._actuator().move_rel(distance)
            return self._target_pos

    def get_target(self):
        """ Get the current target position of the scanner hardware
        (i.e. the "theoretical" position).

        @return dict: current target position per axis.
        """
        with self._thread_lock:
            return self._target_pos

    def get_position(self):
        """ Get a snapshot of the actual scanner position (i.e. from position feedback sensors).
        For the same target this value can fluctuate according to the scanners positioning accuracy.

        For scanning devices that do not have position feedback sensors, simply return the target
        position (see also: ScanningProbeInterface.get_target).

        @return dict: current position per axis.
        """
        with self._thread_lock:
            return self._actuator().get_pos()

    def start_scan(self):

        with self._thread_lock:
            try:

                if self._scan_data is None:
                    self.log.error('Scan Data is None. Scan settings need to be configured before starting')
                    return -1

                if self.module_state() == 'locked':
                    self.log.error('Cannot start a scan while scanning probe is already running')
                    return -1

                self._scan_data.new_scan()
                #self.log.debug(f"New scan data: {self._scan_data.data}, position {self._scan_data._position_data}")
                self._stored_target_pos = self.get_target().copy()
                self._scan_data.scanner_target_at_start = self._stored_target_pos

                self.module_state.lock()

                self._pointer = 0

                first_scan_position = {ax: pos[0] for ax, pos
                                       in zip(self.scan_settings['axes'], self.scan_settings['range'])}

                self._actuator().move_abs(first_scan_position)

                target_pos_vec = self._pos_dict_to_vec(self._target_pos)

                # Terminate follow loop if target is reached
                # while True:
                #     current_pos_vec = self._pos_dict_to_vec(self.get_position())
                #     connecting_vec = target_pos_vec - current_pos_vec
                #     distance_to_target = np.linalg.norm(connecting_vec)
                #     time.sleep(int(round(1 / self._current_scan_frequency)))
                #     if distance_to_target < self._scanner_distance_atol:
                #         self._actuator().move_abs(first_scan_position)
                #         break

                self._sigStartScanning.emit()

            except Exception:
                self.module_state.unlock()
                self.log.error("Scan failed cyka blyat ! You have to do much better !")
                return -1

            #self._start_scanning_timer.start(int(round(1/self._current_scan_frequency))*1000)
            return 0

    def _update_scanning_path(self):

        with self._thread_lock:
            if self._scan_data.scan_dimension == 1:

                h_res = self._scan_data.scan_resolution[0]
                h_min, h_max = self._scan_data.scan_range[0]
                forward_hline_path = np.linspace(h_min, h_max, h_res)
                backward_hline_path = np.linspace(h_max, h_min, self.__backwards_line_resolution)
                self._horizontal_path = np.concatenate((forward_hline_path, backward_hline_path))
                self._vertical_path = None

            elif self._scan_data.scan_dimension == 2:

                h_res = self._scan_data.scan_resolution[0]
                h_min, h_max = self._scan_data.scan_range[0]
                forward_hline_path = np.linspace(h_min, h_max, h_res)
                backward_hline_path = np.linspace(h_max, h_min, self.__backwards_line_resolution)
                h_range = np.concatenate((forward_hline_path, backward_hline_path))

                v_res = self._scan_data.scan_resolution[0]
                v_min, v_max = self._scan_data.scan_range[1]
                v_range = np.linspace(v_min, v_max, v_res)

                h_path, v_path = np.meshgrid(h_range, v_range)

                self._horizontal_path = h_path.flatten()
                self._vertical_path = v_path.flatten()

    def _scan_point(self):
        """ Scans a line and returns the counts on that line.

        @param float[][4] line_path: array of 4-part tuples defining the voltage points
        @param bool pixel_clock: whether we need to output a pixel clock for this line

        @return float[]: the photon counts per second
        """
        with self._thread_lock:
            self.log.info("F")
            if self.module_state() != 'locked':
                return
            self.log.info("G")
            if self._scan_data.scan_dimension == 1:

                if self._pointer >= len(self._horizontal_path):
                    self.log.info('Scan finished')
                    self.stop_scan()
                    return

                position = {self._scan_data.scan_axes[0]: self._horizontal_path[self._pointer]}

            if self._scan_data.scan_dimension == 2:

                if self._pointer >= len(self._horizontal_path):
                    self.log.info('Scan finished')
                    self.stop_scan()
                    return

                position = {self._scan_data.scan_axes[0]: self._horizontal_path[self._pointer],
                            self._scan_data.scan_axes[1]: self._vertical_path[self._pointer]}

            self._actuator().move_abs(position)
            time.sleep(int(round(1 / self._current_scan_frequency)))
            if self._advanced_scan:
                data = self._advanced_scan_function(pointer=self._pointer)
            else:
                data = self._detector().get_data()

            if not isinstance(data, dict):
                self.log.error('The data returned by the detector are not a dictionary.')
                self.stop_scan()
                return

            self.raw_data_container.fill_container({ch:np.array([d]) for ch, d in data.items()})
            self._scan_data.data = self.raw_data_container.forwards_data()

            self._pointer += 1
            self._sigStartScanning.emit()

    def stop_scan(self):
        """

        @return bool: Failure indicator (fail=True)
        """
        with self._thread_lock:
            if self._actuator().module_state == 'locked':
                self._actuator().abort()
                # self.log.debug("Frame stopped")

            self._pointer = 0
            #self._start_scanning_timer.stop()
            self.module_state.unlock()
            self.move_absolute(self._stored_target_pos)
            self._stored_target_pos = dict()

    def get_scan_data(self):
        """

        @return (bool, ScanData): Failure indicator (fail=True), ScanData instance used in the scan
        """
        with self._thread_lock:
            if self._scan_data is None:
                raise RuntimeError('ScanData is not yet configured, please call "configure_scan" first')
            try:
                return self._scan_data.copy()
            except:
                self.log.exception("")

    def emergency_stop(self):
        """

        @return:
        """
        with self._thread_lock:
            self._actuator().abort()
            return False

    @property
    def scan_settings(self):
        with self._thread_lock:
            settings = {'axes': tuple(self._current_scan_axes),
                        'range': tuple(self._current_scan_ranges),
                        'resolution': tuple(self._current_scan_resolution),
                        'frequency': self._current_scan_frequency}
            return settings

    def _pos_dict_to_vec(self, position):

        pos_list = [el[1] for el in sorted(position.items())]
        return np.asarray(pos_list)

    def config_advanced_scan(self, scan_function, scan_channels):
        """ Set the function to use at each point of the scan and the function giving the related channels name.

        @param (function) scan_point_function : function taking in argument the scanning index and position and returning
         a ndarray of acquired data.
        @param (function) scan_point_channels : function returning a list of channels with the same length than the array
        returned by scan_point_function.

        """
        with self._thread_lock:
            self._advanced_scan_function = scan_function
            self._advanced_scan_channels = scan_channels
            self._advanced_scan = True

    @property
    def advanced_scan(self):
        return self._advanced_scan

    @advanced_scan.setter
    def advanced_scan(self, advanced_scan):
        self._advanced_scan = advanced_scan

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