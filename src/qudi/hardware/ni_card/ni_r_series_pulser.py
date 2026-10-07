# -*- coding: utf-8 -*-

"""
This file contains the qudi hardware module to use a National Instruments R-series FPGA
target (e.g. PXI-7852R) as a digital pulse generator implementing the PulserInterface.

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
IMPORTANT HARDWARE NOTE
-----------------------------------------------------------------------------------------------
Like "ni_r_series_instream.py", this module talks to the FPGA through the "nifpga" Python
package and a user-compiled LabVIEW FPGA bitfile (.lvbitx). An R-series target has no built-in
pulse generator personality, so the bitfile must implement the FPGA<->host interface below.
All register/FIFO names are configurable.

Pulse patterns are stored on the FPGA as a run-length encoded instruction list: one
instruction per change of the digital output state. The host converts the sampled waveforms
produced by the qudi pulsed logic into these instructions, so the FPGA memory only limits the
number of state changes and not the total pattern duration.

Instruction format (one U64 element per instruction):
    bits 63..32: digital output state mask (bit 0 -> 'd_ch1', bit 1 -> 'd_ch2', ...)
    bits 31..0:  duration of this state in FPGA clock ticks

Required FPGA<->host interface:
  * Host-to-target DMA FIFO of U64 (`pattern_fifo`, default 'Pattern').
  * Bool CONTROL `load_register` (default 'Load'). While False, the FPGA keeps its memory write
    address at 0. While True, the FPGA pops every element arriving in the pattern FIFO and
    writes it into block memory at the current write address, then increments the address.
  * U32 INDICATOR `loaded_count_register` (default 'Loaded instructions') mirroring the current
    memory write address, i.e. the number of instructions received since Load went True. The
    host uses it to verify the upload is complete.
  * U32 CONTROL `pattern_length_register` (default 'Pattern length'): number of valid
    instructions in memory.
  * Bool CONTROL `run_register` (default 'Run'). On a False->True transition the FPGA starts
    playing instructions 0 ... Pattern length - 1 from memory and loops back to instruction 0
    after the last one, for as long as Run stays True. For each instruction it drives the state
    mask onto the digital output lines and holds it for the given number of ticks. When Run is
    False, all outputs are driven low.

Implementation hints for the playback loop (single-cycle timed loop at the FPGA base clock):
  * Prefetch the next instruction from block memory while the current one is being held, so
    consecutive instructions follow each other without dead time. Block memory reads have a
    latency of a few ticks, hence instructions shorter than `min_instruction_ticks` are rejected
    by this module. Set that option to the true minimum of your implementation.
  * Instructions longer than 2^32 - 1 ticks are split by the host into several instructions.

Sharing the FPGA with "ni_r_series_instream.py":
  An FPGA target can only run one bitfile at a time, so pulse generation and photon counting
  must be compiled into the same bitfile (as parallel loops) to be used together. Both modules
  open their own nifpga Session to the same bitfile. Only one of them should reset the FPGA on
  activation, otherwise the second one wipes the state set up by the first. This module does not
  reset the FPGA by default (`reset_on_activate: False`).

Example config for copy-paste:

nicard_pulser:
    module.Class: 'ni_card.ni_r_series_pulser.NIRSeriesFPGAPulser'
    options:
        resource_name: 'RIO0'  # as shown for your target in NI MAX
        bitfile_path: 'C:/qudi_fpga_bitfiles/Pulser.lvbitx'  # the compiled bitfile
        fpga_clock_frequency: 40e6  # optional, clock the instruction durations are counted in
        digital_channel_count: 8  # optional, number of output lines in the state mask (max 32)
        digital_high_level: 3.3  # optional, fixed digital output high level in V
        min_instruction_ticks: 4  # optional, shortest state duration the FPGA can play
        max_instructions: 4096  # optional, size of the instruction block memory
        max_sample_rate_divider: 1000  # optional, slowest sample rate = clock / divider
        pattern_fifo: 'Pattern'  # optional
        load_register: 'Load'  # optional
        loaded_count_register: 'Loaded instructions'  # optional
        pattern_length_register: 'Pattern length'  # optional
        run_register: 'Run'  # optional
        reset_on_activate: False  # optional
        read_write_timeout: 10  # optional, in seconds
        activation_config:  # optional, defaults to a single config 'all' with every channel
            'all': ['d_ch1', 'd_ch2', 'd_ch3', 'd_ch4', 'd_ch5', 'd_ch6', 'd_ch7', 'd_ch8']
            'laser_mw_apd': ['d_ch1', 'd_ch2', 'd_ch3']
"""

import time
import numpy as np
import nifpga

from qudi.core.configoption import ConfigOption
from qudi.util.mutex import Mutex
from qudi.util.constraints import ScalarConstraint
from qudi.interface.pulser_interface import PulserInterface, PulserConstraints, SequenceOption


def _legacy_scalar_constraint(default, bounds, increment=None, enforce_int=False):
    """ Creates a ScalarConstraint that additionally exposes the legacy "min", "max" and "step"
    attributes read by the pulsed logic modules.
    """
    constraint = ScalarConstraint(default=default,
                                  bounds=bounds,
                                  increment=increment,
                                  enforce_int=enforce_int)
    constraint.min = constraint.minimum
    constraint.max = constraint.maximum
    constraint.step = 0 if increment is None else increment
    return constraint


class NIRSeriesFPGAPulser(PulserInterface):
    """
    A National Instruments R-series FPGA target (e.g. PXI-7852R) driven through a user-supplied
    LabVIEW FPGA bitfile, used as a purely digital pulse generator.

    See the module docstring for the required FPGA register/FIFO interface and an example config.
    """

    _resource_name = ConfigOption(name='resource_name', default='RIO0', missing='warn')
    _bitfile_path = ConfigOption(name='bitfile_path', default=None, missing='error')
    _fpga_clock_frequency = ConfigOption('fpga_clock_frequency',
                                         default=40e6,
                                         missing='nothing',
                                         constructor=lambda x: float(x))
    _digital_channel_count = ConfigOption('digital_channel_count', default=8, missing='nothing')
    _digital_high_level = ConfigOption('digital_high_level',
                                       default=3.3,
                                       missing='nothing',
                                       constructor=lambda x: float(x))
    _min_instruction_ticks = ConfigOption('min_instruction_ticks', default=4, missing='nothing')
    _max_instructions = ConfigOption('max_instructions', default=4096, missing='nothing')
    _max_sample_rate_divider = ConfigOption('max_sample_rate_divider',
                                            default=1000,
                                            missing='nothing')
    _pattern_fifo_name = ConfigOption('pattern_fifo', default='Pattern', missing='nothing')
    _load_register = ConfigOption('load_register', default='Load', missing='nothing')
    _loaded_count_register = ConfigOption('loaded_count_register',
                                          default='Loaded instructions',
                                          missing='nothing')
    _pattern_length_register = ConfigOption('pattern_length_register',
                                            default='Pattern length',
                                            missing='nothing')
    _run_register = ConfigOption('run_register', default='Run', missing='nothing')
    _reset_on_activate = ConfigOption('reset_on_activate', default=False, missing='nothing')
    _rw_timeout = ConfigOption('read_write_timeout', default=10, missing='nothing')
    _activation_config = ConfigOption('activation_config', default=None, missing='nothing')

    _MAX_TICKS_PER_INSTRUCTION = 2**32 - 1

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = Mutex()
        self._session = None
        self._constraints = None
        self._digital_channels = tuple()
        self._sample_rate_divider = 1
        self._active_channels = frozenset()
        self._is_running = False
        # waveform name -> (state masks, run lengths in samples, total number of samples)
        self._waveforms = dict()
        # waveform name -> [list of mask arrays, list of length arrays, samples written so far]
        self._waveform_in_progress = dict()
        self._loaded_waveform = ''

    def on_activate(self):
        """ Opens the FPGA session and performs sanity checks. """
        if not self._bitfile_path:
            raise ValueError('"bitfile_path" must be specified in config. It must point to a '
                             'compiled LabVIEW FPGA bitfile (.lvbitx) implementing the '
                             'FPGA<->host interface documented in ni_r_series_pulser.py.')
        if not 1 <= self._digital_channel_count <= 32:
            raise ValueError('Config option "digital_channel_count" must be in range [1, 32].')

        try:
            # no_run: do not restart the VI if another module (e.g. the instreamer) already runs it
            self._session = nifpga.Session(bitfile=self._bitfile_path,
                                           resource=self._resource_name,
                                           no_run=True)
            if self._reset_on_activate:
                self._session.reset()
            # Harmless if the VI is already running (nifpga only issues a warning)
            self._session.run()
        except Exception as err:
            raise RuntimeError(
                f'Could not open nifpga Session for resource "{self._resource_name}" with '
                f'bitfile "{self._bitfile_path}". Is the FPGA card connected and the resource '
                f'name correct?'
            ) from err

        required_registers = (self._load_register,
                              self._loaded_count_register,
                              self._pattern_length_register,
                              self._run_register)
        for reg_name in required_registers:
            if reg_name not in self._session.registers:
                self._terminate_session()
                raise ValueError(f'Register "{reg_name}" not found in bitfile. Available '
                                 f'registers: {list(self._session.registers)}')
        if self._pattern_fifo_name not in self._session.fifos:
            self._terminate_session()
            raise ValueError(f'FIFO "{self._pattern_fifo_name}" not found in bitfile. Available '
                             f'FIFOs: {list(self._session.fifos)}')

        self._digital_channels = tuple(
            f'd_ch{i:d}' for i in range(1, self._digital_channel_count + 1)
        )
        self._constraints = self._create_constraints()
        self._sample_rate_divider = 1
        self._active_channels = next(iter(self._constraints.activation_config.values()))
        self._waveforms = dict()
        self._waveform_in_progress = dict()
        self._loaded_waveform = ''

        self._session.registers[self._run_register].write(False)
        self._session.registers[self._load_register].write(False)
        self._is_running = False

    def on_deactivate(self):
        """ Stops the pulse output and closes the FPGA session. """
        self._terminate_session()

    def _create_constraints(self):
        clock = self._fpga_clock_frequency
        constraints = PulserConstraints()
        constraints.sample_rate = _legacy_scalar_constraint(
            default=clock,
            bounds=(clock / self._max_sample_rate_divider, clock)
        )
        constraints.a_ch_amplitude = _legacy_scalar_constraint(default=0., bounds=(0., 0.))
        constraints.a_ch_offset = _legacy_scalar_constraint(default=0., bounds=(0., 0.))
        constraints.d_ch_low = _legacy_scalar_constraint(default=0., bounds=(0., 0.))
        constraints.d_ch_high = _legacy_scalar_constraint(
            default=self._digital_high_level,
            bounds=(self._digital_high_level, self._digital_high_level)
        )
        # The pattern is stored run-length encoded, so the number of samples is not the limiting
        # factor. The number of state changes is checked against "max_instructions" on loading.
        constraints.waveform_length = _legacy_scalar_constraint(default=1,
                                                                bounds=(1, 2**48),
                                                                increment=1,
                                                                enforce_int=True)
        constraints.waveform_num = _legacy_scalar_constraint(default=1,
                                                             bounds=(1, 1),
                                                             increment=1,
                                                             enforce_int=True)
        constraints.sequence_num = _legacy_scalar_constraint(default=0,
                                                             bounds=(0, 0),
                                                             increment=1,
                                                             enforce_int=True)
        constraints.subsequence_num = _legacy_scalar_constraint(default=0,
                                                                bounds=(0, 0),
                                                                increment=1,
                                                                enforce_int=True)
        constraints.sequence_steps = _legacy_scalar_constraint(default=0,
                                                               bounds=(0, 0),
                                                               increment=1,
                                                               enforce_int=True)
        constraints.repetitions = _legacy_scalar_constraint(default=0,
                                                            bounds=(0, 0),
                                                            increment=1,
                                                            enforce_int=True)
        constraints.event_triggers = list()
        constraints.flags = list()
        constraints.sequence_option = SequenceOption.NON

        if self._activation_config:
            activation_config = dict()
            for name, channels in self._activation_config.items():
                invalid = set(channels).difference(self._digital_channels)
                if invalid:
                    raise ValueError(f'Invalid channels {invalid} in activation_config "{name}". '
                                     f'Valid channels are {self._digital_channels}.')
                activation_config[name] = frozenset(channels)
        else:
            activation_config = {'all': frozenset(self._digital_channels)}
        constraints.activation_config = activation_config
        return constraints

    def get_constraints(self):
        """ Retrieve the hardware constrains from the Pulsing device.

        @return constraints object: object with pulser constraints as attributes.
        """
        return self._constraints

    def pulser_on(self):
        """ Switches the pulsing device on.

        @return int: error code (0:OK, -1:error)
        """
        with self._thread_lock:
            if not self._loaded_waveform:
                self.log.error('Unable to start pulser. No waveform loaded.')
                return -1
            if self._is_running:
                return 0
            self._session.registers[self._run_register].write(True)
            self._is_running = True
            return 0

    def pulser_off(self):
        """ Switches the pulsing device off.

        @return int: error code (0:OK, -1:error)
        """
        with self._thread_lock:
            self._session.registers[self._run_register].write(False)
            self._is_running = False
            return 0

    def load_waveform(self, load_dict):
        """ Uploads a waveform previously written with write_waveform to the FPGA memory, making it
        ready to play.

        @param dict|list load_dict: a dictionary with keys being the channel index and values being
                                    the waveform name, or a list of waveform names. All digital
                                    channels share a single waveform on this device, so only one
                                    waveform name may be given.

        @return dict: Dictionary containing the actually loaded waveforms per channel.
        """
        if isinstance(load_dict, dict):
            names = set(load_dict.values())
        else:
            names = set(load_dict)
        if len(names) != 1:
            self.log.error(f'Exactly one waveform can be loaded at a time on this device. '
                           f'Received {names}.')
            return self.get_loaded_assets()[0]
        name = names.pop()

        with self._thread_lock:
            if self._is_running:
                self.log.error('Unable to load waveform while the pulser is running.')
                return self._loaded_assets()
            if name not in self._waveforms:
                self.log.error(f'Loading failed. Waveform "{name}" has not been written.')
                return self._loaded_assets()

            try:
                instructions = self._encode_instructions(*self._waveforms[name][:2])
                self._upload_instructions(instructions)
            except Exception:
                self._loaded_waveform = ''
                self.log.exception(f'Loading waveform "{name}" to FPGA failed.')
                return self._loaded_assets()
            self._loaded_waveform = name
            return self._loaded_assets()

    def load_sequence(self, sequence_name):
        """ Sequences are not supported by this device.

        @return dict: Dictionary containing the actually loaded waveforms per channel.
        """
        self.log.error('Sequence mode is not supported by the NI R-series FPGA pulser.')
        return self.get_loaded_assets()[0]

    def get_loaded_assets(self):
        """ Retrieve the currently loaded asset names for each active channel of the device.

        @return (dict, str): Dictionary with keys being the channel number and values being the
                             respective asset loaded into the channel,
                             string describing the asset type ('waveform' or 'sequence')
        """
        with self._thread_lock:
            assets = self._loaded_assets()
        return assets, 'waveform' if assets else ''

    def clear_all(self):
        """ Clears all loaded waveforms from the pulse generators RAM/workspace.

        @return int: error code (0:OK, -1:error)
        """
        with self._thread_lock:
            if self._is_running:
                self.log.error('Unable to clear waveforms while the pulser is running.')
                return -1
            self._waveforms = dict()
            self._waveform_in_progress = dict()
            self._loaded_waveform = ''
            return 0

    def get_status(self):
        """ Retrieves the status of the pulsing hardware

        @return (int, dict): tuple with an integer value of the current status and a corresponding
                             dictionary containing status description for all the possible status
                             variables of the pulse generator hardware.
        """
        status_dict = {-1: 'Failed Request or Communication',
                       0: 'Device has stopped, but can receive commands.',
                       1: 'Device is active and running.'}
        if self._session is None:
            return -1, status_dict
        return int(self._is_running), status_dict

    def get_sample_rate(self):
        """ Get the sample rate of the pulse generator hardware

        @return float: The current sample rate of the device (in Hz)
        """
        return self._fpga_clock_frequency / self._sample_rate_divider

    def set_sample_rate(self, sample_rate):
        """ Set the sample rate of the pulse generator hardware. The sample rate is the FPGA clock
        frequency divided by an integer, so the closest possible value is set.

        @param float sample_rate: The sampling rate to be set (in Hz)

        @return float: the sample rate returned from the device (in Hz).
        """
        with self._thread_lock:
            if self._is_running:
                self.log.error('Unable to set sample rate while the pulser is running.')
                return self.get_sample_rate()
            divider = int(round(self._fpga_clock_frequency / sample_rate))
            self._sample_rate_divider = min(max(divider, 1), self._max_sample_rate_divider)
            new_rate = self.get_sample_rate()
            if not np.isclose(new_rate, sample_rate):
                self.log.warning(f'Requested sample rate {sample_rate:.6e} Hz is not an integer '
                                 f'divider of the FPGA clock. Set to {new_rate:.6e} Hz instead.')
            return new_rate

    def get_analog_level(self, amplitude=None, offset=None):
        """ This device has no analog channels.

        @return: (dict, dict): two empty dicts
        """
        return dict(), dict()

    def set_analog_level(self, amplitude=None, offset=None):
        """ This device has no analog channels.

        @return (dict, dict): two empty dicts
        """
        if amplitude or offset:
            self.log.warning('NI R-series FPGA pulser has no analog channels. Ignoring levels.')
        return dict(), dict()

    def get_digital_level(self, low=None, high=None):
        """ Retrieve the digital low and high level of the provided/all channels. These levels are
        fixed by the FPGA output hardware.

        @param list low: optional, channels to get the low level for
        @param list high: optional, channels to get the high level for

        @return: (dict, dict): low and high levels in volts per channel descriptor
        """
        low = self._digital_channels if not low else low
        high = self._digital_channels if not high else high
        return ({ch: 0. for ch in low if ch in self._digital_channels},
                {ch: self._digital_high_level for ch in high if ch in self._digital_channels})

    def set_digital_level(self, low=None, high=None):
        """ Digital levels are fixed by the FPGA output hardware and can not be changed.

        @return (dict, dict): the actual low and high levels for ALL digital channels.
        """
        if low or high:
            self.log.warning('Digital levels of the NI R-series FPGA pulser are fixed '
                             f'(low: 0 V, high: {self._digital_high_level} V).')
        return self.get_digital_level()

    def get_active_channels(self, ch=None):
        """ Get the active channels of the pulse generator hardware.

        @param list ch: optional, channels to get the state for. All channels if None.

        @return dict: channel descriptor -> bool (active)
        """
        channels = self._digital_channels if not ch else ch
        return {chnl: chnl in self._active_channels
                for chnl in channels if chnl in self._digital_channels}

    def set_active_channels(self, ch=None):
        """ Set the active/inactive channels for the pulse generator hardware. The resulting set
        of active channels must be one of the activation configs in the constraints, otherwise
        the channel states remain unchanged. Inactive channels are kept low during playback.

        @param dict ch: channel descriptor -> bool (True: activate, False: deactivate)

        @return dict: with the actual set values for ALL digital channels
        """
        if ch:
            with self._thread_lock:
                if self._is_running:
                    self.log.error('Unable to change active channels while the pulser is running.')
                    return self.get_active_channels()
                new_active = set(self._active_channels)
                for chnl, state in ch.items():
                    if chnl not in self._digital_channels:
                        self.log.error(f'Unknown channel "{chnl}". Active channels unchanged.')
                        return self.get_active_channels()
                    if state:
                        new_active.add(chnl)
                    else:
                        new_active.discard(chnl)
                new_active = frozenset(new_active)
                if new_active not in self._constraints.activation_config.values():
                    self.log.error(f'Channel activation {sorted(new_active)} is not part of the '
                                   f'available activation configs. Active channels unchanged.')
                    return self.get_active_channels()
                self._active_channels = new_active
        return self.get_active_channels()

    def write_waveform(self, name, analog_samples, digital_samples, is_first_chunk, is_last_chunk,
                       total_number_of_samples):
        """ Write a new waveform or append samples to an already existing waveform. The waveform is
        kept in host memory in run-length encoded form and only uploaded to the FPGA by
        load_waveform.

        @param str name: the name of the waveform to be created/append to
        @param dict analog_samples: must be empty, this device has no analog channels
        @param dict digital_samples: keys are the generic digital channel names (i.e. 'd_ch1') and
                                     values are 1D numpy arrays of type bool containing the
                                     channel states.
        @param bool is_first_chunk: Flag indicating if it is the first chunk to write.
        @param bool is_last_chunk: Flag indicating if it is the last chunk to write.
        @param int total_number_of_samples: The number of sample points for the entire waveform

        @return (int, list): Number of samples written (-1 indicates failed process) and list of
                             created waveform names
        """
        if analog_samples:
            self.log.error('NI R-series FPGA pulser has no analog channels. Write failed.')
            return -1, list()
        if not digital_samples:
            self.log.error('No digital samples passed to write_waveform.')
            return -1, list()
        lengths = {len(samples) for samples in digital_samples.values()}
        if len(lengths) != 1:
            self.log.error('Unequal length of sample arrays for different channels.')
            return -1, list()
        number_of_samples = lengths.pop()
        invalid = set(digital_samples).difference(self._digital_channels)
        if invalid:
            self.log.error(f'Invalid digital channels {invalid} encountered. Write failed.')
            return -1, list()

        with self._thread_lock:
            if is_first_chunk or name not in self._waveform_in_progress:
                if not is_first_chunk:
                    self.log.error(f'Unable to append to waveform "{name}". Writing has not been '
                                   f'started with is_first_chunk=True.')
                    return -1, list()
                self._waveforms.pop(name, None)
                if self._loaded_waveform == name:
                    self._loaded_waveform = ''
                self._waveform_in_progress[name] = [list(), list(), 0]

            masks, run_lengths = self._samples_to_runs(digital_samples, number_of_samples)
            in_progress = self._waveform_in_progress[name]
            # Merge the first run of this chunk into the last run of the previous chunk if the
            # output state does not change across the chunk border
            if in_progress[0] and masks.size and in_progress[0][-1][-1] == masks[0]:
                in_progress[1][-1][-1] += run_lengths[0]
                masks, run_lengths = masks[1:], run_lengths[1:]
            if masks.size:
                in_progress[0].append(masks)
                in_progress[1].append(run_lengths)
            in_progress[2] += number_of_samples

            if is_last_chunk:
                del self._waveform_in_progress[name]
                if in_progress[2] != total_number_of_samples:
                    self.log.error(f'Number of samples written to waveform "{name}" '
                                   f'({in_progress[2]:d}) does not match the expected total '
                                   f'({total_number_of_samples:d}).')
                    return -1, list()
                self._waveforms[name] = (np.concatenate(in_progress[0]),
                                         np.concatenate(in_progress[1]),
                                         in_progress[2])
        return number_of_samples, [name]

    def write_sequence(self, name, sequence_parameters):
        """ Sequences are not supported by this device.

        @return: int, number of sequence steps written (-1 indicates failed process)
        """
        self.log.error('Sequence mode is not supported by the NI R-series FPGA pulser.')
        return -1

    def get_waveform_names(self):
        """ Retrieve the names of all written waveforms.

        @return list: List of all written waveform name strings.
        """
        with self._thread_lock:
            return list(self._waveforms)

    def get_sequence_names(self):
        """ Sequences are not supported by this device.

        @return list: empty list
        """
        return list()

    def delete_waveform(self, waveform_name):
        """ Delete the waveform with name "waveform_name" from the device memory.

        @param str waveform_name: The name of the waveform to be deleted
                                  Optionally a list of waveform names can be passed.

        @return list: a list of deleted waveform names.
        """
        names = [waveform_name] if isinstance(waveform_name, str) else list(waveform_name)
        deleted = list()
        with self._thread_lock:
            for name in names:
                if name in self._waveforms:
                    if name == self._loaded_waveform and self._is_running:
                        self.log.error(f'Unable to delete waveform "{name}" while it is playing.')
                        continue
                    del self._waveforms[name]
                    if name == self._loaded_waveform:
                        self._loaded_waveform = ''
                    deleted.append(name)
        return deleted

    def delete_sequence(self, sequence_name):
        """ Sequences are not supported by this device.

        @return list: empty list
        """
        return list()

    def get_interleave(self):
        """ Interleave is not available on this device.

        @return bool: False
        """
        return False

    def set_interleave(self, state=False):
        """ Interleave is not available on this device.

        @return bool: False
        """
        if state:
            self.log.warning('Interleave is not available on the NI R-series FPGA pulser.')
        return False

    def reset(self):
        """ Stops the pulse output and forgets the loaded waveform. The FPGA itself is not reset,
        since it may be shared with other modules (e.g. ni_r_series_instream).

        @return int: error code (0:OK, -1:error)
        """
        self.pulser_off()
        with self._thread_lock:
            self._loaded_waveform = ''
        return 0

    # =============================================================================================

    def _loaded_assets(self):
        """ Caller must hold the thread lock. """
        return {1: self._loaded_waveform} if self._loaded_waveform else dict()

    def _samples_to_runs(self, digital_samples, number_of_samples):
        """ Combines the digital channel samples into output state masks and compresses them into
        runs of constant state. Inactive channels are kept low.

        @return (np.ndarray, np.ndarray): state mask and length in samples for each run
        """
        masks = np.zeros(number_of_samples, dtype=np.uint64)
        for chnl, samples in digital_samples.items():
            if chnl in self._active_channels:
                bit = np.uint64(self._digital_channels.index(chnl))
                masks |= np.asarray(samples, dtype=bool).astype(np.uint64) << bit
        if number_of_samples == 0:
            return masks, np.zeros(0, dtype=np.uint64)
        run_starts = np.concatenate(([0], np.flatnonzero(np.diff(masks)) + 1))
        run_lengths = np.diff(np.append(run_starts, number_of_samples)).astype(np.uint64)
        return masks[run_starts], run_lengths

    def _encode_instructions(self, masks, run_lengths):
        """ Converts runs (in samples) into packed U64 FPGA instructions (in clock ticks) for the
        current sample rate.

        @return np.ndarray: packed uint64 instructions
        """
        ticks = run_lengths.astype(np.uint64) * np.uint64(self._sample_rate_divider)
        too_short = ticks < self._min_instruction_ticks
        if np.any(too_short):
            shortest = int(ticks[too_short].min())
            raise ValueError(
                f'Waveform contains a pulse/pause of {shortest:d} FPGA clock ticks '
                f'({shortest / self._fpga_clock_frequency:.3e} s) which is shorter than the '
                f'minimum of {self._min_instruction_ticks:d} ticks.'
            )
        # Split runs that exceed the duration register width into several instructions
        max_ticks = np.uint64(self._MAX_TICKS_PER_INSTRUCTION)
        repeats = ((ticks + max_ticks - np.uint64(1)) // max_ticks).astype(np.int64)
        instr_masks = np.repeat(masks, repeats)
        instr_ticks = np.repeat(ticks, repeats)
        if np.any(repeats > 1):
            # Last instruction of each split run holds the remainder, all others hold max_ticks
            last_index = np.cumsum(repeats) - 1
            remainder = ticks - (repeats.astype(np.uint64) - np.uint64(1)) * max_ticks
            instr_ticks[:] = max_ticks
            instr_ticks[last_index] = remainder
        if instr_ticks.size > self._max_instructions:
            raise ValueError(
                f'Waveform requires {instr_ticks.size:d} FPGA instructions (one per output state '
                f'change) but the FPGA memory only holds {self._max_instructions:d}.'
            )
        return (instr_masks << np.uint64(32)) | instr_ticks

    def _upload_instructions(self, instructions):
        """ Writes the instructions into the FPGA block memory using the Load handshake documented
        in the module docstring. Caller must hold the thread lock.
        """
        registers = self._session.registers
        fifo = self._session.fifos[self._pattern_fifo_name]
        number_of_instructions = int(instructions.size)

        registers[self._run_register].write(False)
        registers[self._load_register].write(False)
        registers[self._pattern_length_register].write(number_of_instructions)
        try:
            fifo.configure(number_of_instructions)
            fifo.start()
            registers[self._load_register].write(True)
            fifo.write(instructions.tolist(), timeout_ms=int(self._rw_timeout * 1000))

            deadline = time.monotonic() + self._rw_timeout
            while registers[self._loaded_count_register].read() < number_of_instructions:
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f'FPGA only acknowledged '
                        f'{registers[self._loaded_count_register].read():d} of '
                        f'{number_of_instructions:d} instructions within {self._rw_timeout} s.'
                    )
                time.sleep(0.001)
        finally:
            registers[self._load_register].write(False)
            fifo.stop()

    def _terminate_session(self):
        if self._session is not None:
            try:
                self._session.registers[self._run_register].write(False)
            except Exception:
                pass
            try:
                self._session.close()
            except Exception:
                self.log.exception('Error while trying to close nifpga Session.')
            finally:
                self._session = None
                self._is_running = False
