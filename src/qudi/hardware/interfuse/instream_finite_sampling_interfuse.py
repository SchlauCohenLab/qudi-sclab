# -*- coding: utf-8 -*-

"""
This file contains a qudi interfuse exposing a continuous data streamer (DataInStreamInterface,
e.g. ni_r_series_instream) as a finite sampling input (FiniteSamplingInputInterface), so it can be
used as "data_scanner" by the ODMR logic.

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

-----------------------------------------------------------------------------------------------
SYNCHRONISATION NOTE
-----------------------------------------------------------------------------------------------
Each frame starts the underlying stream, reads <frame_size> samples and stops the stream again.
For ODMR, the microwave source must advance one frequency per sample. With an externally
triggered source (e.g. mw_source_siglent_ssg5000x), the FPGA must therefore output one trigger
pulse per counting window and that line must be wired to the source's trigger input.

If the first counting window(s) of a frame do not correspond to the first frequency (e.g.
because the FPGA emits its trigger at the start rather than the end of a window), use
`discard_samples` to drop that many samples at the start of every frame. In that case the
microwave source receives `discard_samples` additional triggers per frame, so check the
alignment with a known resonance before relying on the data.

Example config for copy-paste:

odmr_sampler:
    module.Class: 'interfuse.instream_finite_sampling_interfuse.InStreamFiniteSamplingInterfuse'
    options:
        discard_samples: 0  # optional, samples to drop at the start of each frame
    connect:
        streamer: nicard_apd_instreamer
"""

import numpy as np

from qudi.core.configoption import ConfigOption
from qudi.core.connector import Connector
from qudi.util.mutex import RecursiveMutex
from qudi.interface.finite_sampling_input_interface import FiniteSamplingInputInterface
from qudi.interface.finite_sampling_input_interface import FiniteSamplingInputConstraints


class InStreamFiniteSamplingInterfuse(FiniteSamplingInputInterface):
    """ Exposes a DataInStreamInterface hardware as FiniteSamplingInputInterface.

    See the module docstring for synchronisation details and an example config.
    """

    _streamer = Connector(name='streamer', interface='DataInStreamInterface')

    _discard_samples = ConfigOption('discard_samples', default=0, missing='nothing')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._thread_lock = RecursiveMutex()
        self._constraints = None
        self._active_channels = frozenset()
        self._sample_rate = -1.
        self._frame_size = -1
        self._samples_pending = 0
        self._continuous_mode = None

    def on_activate(self):
        streamer_constraints = self._streamer().constraints
        modes = [mode for mode in streamer_constraints.streaming_modes
                 if getattr(mode, 'name', '') == 'CONTINUOUS']
        if not modes:
            raise RuntimeError('Connected data streamer does not support CONTINUOUS streaming.')
        self._continuous_mode = modes[0]

        buffer_size = streamer_constraints.channel_buffer_size
        self._constraints = FiniteSamplingInputConstraints(
            channel_units=streamer_constraints.channel_units,
            frame_size_limits=(1, buffer_size.maximum - int(self._discard_samples)),
            sample_rate_limits=streamer_constraints.sample_rate.bounds
        )
        self._active_channels = frozenset(self._constraints.channel_names)
        self._sample_rate = streamer_constraints.sample_rate.default
        self._frame_size = 1
        self._samples_pending = 0

    def on_deactivate(self):
        self.stop_buffered_acquisition()

    @property
    def constraints(self):
        return self._constraints

    @property
    def active_channels(self):
        return self._active_channels

    @property
    def sample_rate(self):
        return self._sample_rate

    @property
    def frame_size(self):
        return self._frame_size

    @property
    def samples_in_buffer(self):
        with self._thread_lock:
            if self.module_state() != 'locked':
                return 0
            return min(self._streamer().available_samples, self._samples_pending)

    def set_sample_rate(self, rate):
        with self._thread_lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Unable to set sample rate while acquisition is running.')
            if not self._constraints.sample_rate_in_range(rate)[0]:
                raise ValueError(f'Sample rate {rate} Hz out of bounds '
                                 f'{self._constraints.sample_rate_limits}.')
            self._sample_rate = float(rate)

    def set_active_channels(self, channels):
        with self._thread_lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Unable to set active channels while acquisition is running.')
            channels = frozenset(channels)
            invalid = channels.difference(self._constraints.channel_names)
            if invalid:
                raise ValueError(f'Invalid channels {invalid}. Valid channels are '
                                 f'{self._constraints.channel_names}.')
            self._active_channels = channels

    def set_frame_size(self, size):
        with self._thread_lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Unable to set frame size while acquisition is running.')
            size = int(round(size))
            if not self._constraints.frame_size_in_range(size)[0]:
                raise ValueError(f'Frame size {size} out of bounds '
                                 f'{self._constraints.frame_size_limits}.')
            self._frame_size = size

    def start_buffered_acquisition(self):
        with self._thread_lock:
            if self.module_state() == 'locked':
                raise RuntimeError('Acquisition already running.')
            streamer = self._streamer()
            # Keep the streamer's channel order so the interleaved data can be unraveled
            channels = [ch for ch in self._constraints.channel_names
                        if ch in self._active_channels]
            buffer_size = max(streamer.constraints.channel_buffer_size.default,
                              self._frame_size + int(self._discard_samples))
            streamer.configure(active_channels=channels,
                               streaming_mode=self._continuous_mode,
                               channel_buffer_size=buffer_size,
                               sample_rate=self._sample_rate)
            self.module_state.lock()
            try:
                streamer.start_stream()
                if self._discard_samples > 0:
                    streamer.read_data(int(self._discard_samples))
            except Exception:
                self._stop_streamer()
                raise
            self._samples_pending = self._frame_size

    def stop_buffered_acquisition(self):
        with self._thread_lock:
            if self.module_state() == 'locked':
                self._stop_streamer()

    def get_buffered_samples(self, number_of_samples=None):
        with self._thread_lock:
            if self.module_state() != 'locked':
                return {ch: np.empty(0) for ch in self._active_channels}
            if number_of_samples is None:
                number_of_samples = self.samples_in_buffer
            elif number_of_samples > self._samples_pending:
                raise ValueError(f'Requested {number_of_samples} samples but only '
                                 f'{self._samples_pending} are pending in this frame.')

            streamer = self._streamer()
            channels = streamer.active_channels
            data, _ = streamer.read_data(number_of_samples)
            data = data.reshape(number_of_samples, len(channels))
            self._samples_pending -= number_of_samples
            if self._samples_pending <= 0:
                self._stop_streamer()
            return {ch: data[:, i].copy() for i, ch in enumerate(channels)}

    def acquire_frame(self, frame_size=None):
        with self._thread_lock:
            if frame_size is None:
                self.start_buffered_acquisition()
                return self.get_buffered_samples(self._frame_size)

            old_frame_size = self._frame_size
            self.set_frame_size(frame_size)
            try:
                self.start_buffered_acquisition()
                return self.get_buffered_samples(self._frame_size)
            finally:
                self.stop_buffered_acquisition()
                self._frame_size = old_frame_size

    def _stop_streamer(self):
        """ Caller must hold the thread lock and the module must be locked. """
        try:
            self._streamer().stop_stream()
        finally:
            self._samples_pending = 0
            self.module_state.unlock()
