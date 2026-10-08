# -*- coding: utf-8 -*-

"""
This file contains the qudi hardware module to control Siglent SSG5000X series RF signal
generators (e.g. SSG5060X-V) via SCPI.

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
SCAN / TRIGGER NOTES
-----------------------------------------------------------------------------------------------
Frequency scans use the instrument's built-in sweep engine (SSG5000X programming guide,
"[:SOURce]:SWEep" subsystem):
  * SamplingOutputMode.EQUIDISTANT_SWEEP -> ":SWEep:TYPE STEP" (start, stop, points)
  * SamplingOutputMode.JUMP_LIST         -> ":SWEep:TYPE LIST" (one list row per frequency)

The whole sweep is armed immediately (":SWEep:SWEep:TRIGger:TYPE AUTO") and every single point
is advanced by a pulse on the rear-panel TRIGGER IN connector
(":SWEep:POINt:TRIGger:TYPE EXT"), which is what the qudi ODMR logic expects. The active edge
is selected with the `trigger_edge` config option.

The instrument dwell time is limited to 10 ms ... 100 s per point. With external point
triggering, the dwell time is set to the shortest allowed value so that the trigger rate, not the
dwell time, paces the scan. The maximum external point trigger rate is not specified in the
programming guide, hence `scan_sample_rate_limits` is a config option. Validate it on your unit
before relying on fast scans.

-----------------------------------------------------------------------------------------------
HARDWARE LIMITS
-----------------------------------------------------------------------------------------------
The SSG5000X SCPI command set has no MIN/MAX limit queries, so the limits are derived from the
model reported by "*IDN?" and the SSG5000X datasheet:
  * Frequency: 9 kHz to 4 GHz (SSG5040X, SSG5040X-V) or 6 GHz (SSG5060X, SSG5060X-V)
  * Level setting range, depending on frequency f:
        9 kHz <= f < 100 kHz:  -110 dBm to  +7 dBm
      100 kHz <= f <   1 MHz:  -110 dBm to +15 dBm
        1 MHz <= f <   4 GHz:  -140 dBm to +26 dBm
        4 GHz <= f <=  6 GHz:  -130 dBm to +24 dBm
    The constraints report the overall range and every CW/scan setting is additionally checked
    against the range of the frequency band(s) it uses.
  * Scan size: 2 to 65535 points for EQUIDISTANT_SWEEP (step sweep), at most 500 points for
    JUMP_LIST (list sweep)
`frequency_limits` and `power_limits` are optional and can only narrow these limits, e.g. to
protect an amplifier connected to the output.

Example config for copy-paste:

mw_source_siglent:
    module.Class: 'microwave.mw_source_siglent_ssg5000x.MicrowaveSiglentSSG5000X'
    options:
        visa_address: 'TCPIP0::192.168.1.100::inst0::INSTR'  # or 'USB0::0xF4EC::...::INSTR'
        comm_timeout: 10  # in seconds
        trigger_edge: 'rising'  # optional, 'rising' or 'falling'
        frequency_limits: [2.5e9, 3.2e9]  # optional, narrows the model frequency range
        power_limits: [-140, 0]  # optional, narrows the model level range (dBm)
        scan_sample_rate_limits: [0.01, 100]  # optional, external point trigger rate in Hz
        reset_on_activate: False  # optional, send *RST on activation
"""

import time
import numpy as np
import pyvisa

from qudi.util.mutex import Mutex
from qudi.core.configoption import ConfigOption
from qudi.interface.microwave_interface import MicrowaveInterface, MicrowaveConstraints
from qudi.util.enums import SamplingOutputMode


class MicrowaveSiglentSSG5000X(MicrowaveInterface):
    """ Hardware control class for Siglent SSG5000X series RF signal generators (e.g. SSG5060X-V).

    See the module docstring for scan/trigger details and an example config.
    """

    _visa_address = ConfigOption('visa_address', missing='error')
    _comm_timeout = ConfigOption('comm_timeout', default=10, missing='warn')
    _trigger_edge = ConfigOption('trigger_edge',
                                 default='rising',
                                 missing='nothing',
                                 constructor=lambda x: str(x).lower())
    _frequency_limits = ConfigOption('frequency_limits', default=None, missing='nothing')
    _power_limits = ConfigOption('power_limits', default=None, missing='nothing')
    _scan_sample_rate_limits = ConfigOption('scan_sample_rate_limits',
                                            default=(0.01, 100),
                                            missing='nothing')
    _reset_on_activate = ConfigOption('reset_on_activate', default=False, missing='nothing')

    # Hardware limits from the SSG5000X datasheet, see module docstring
    _MIN_FREQUENCY = 9e3
    _MODEL_MAX_FREQUENCY = {'SSG5040X': 4e9, 'SSG5060X': 6e9}
    # (band start in Hz, min level in dBm, max level in dBm), each band ends at the next start
    _LEVEL_SETTING_RANGES = ((9e3, -110., 7.),
                             (100e3, -110., 15.),
                             (1e6, -140., 26.),
                             (4e9, -130., 24.))
    _MAX_STEP_SWEEP_POINTS = 65535
    _MAX_LIST_SWEEP_POINTS = 500
    # Dwell time limits per sweep point as stated in the SSG5000X programming guide
    _MIN_DWELL_TIME = 10e-3
    _MAX_DWELL_TIME = 100.

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._thread_lock = Mutex()
        self._rm = None
        self._device = None
        self._model = ''
        self._constraints = None
        self._scan_power = -20.
        self._scan_frequencies = None
        self._scan_mode = SamplingOutputMode.JUMP_LIST
        self._scan_sample_rate = 0.
        self._in_cw_mode = True

    def on_activate(self):
        """ Initialisation performed during activation of the module. """
        if self._trigger_edge not in ('rising', 'falling'):
            raise ValueError(f'Config option "trigger_edge" must be "rising" or "falling". '
                             f'Received "{self._trigger_edge}".')

        self._rm = pyvisa.ResourceManager()
        try:
            self._device = self._rm.open_resource(self._visa_address,
                                                  timeout=int(self._comm_timeout * 1000))
            self._device.read_termination = '\n'
            self._device.write_termination = '\n'
            self._model = self._device.query('*IDN?').strip().split(',')[1]
        except Exception:
            self._rm.close()
            self._rm = None
            self._device = None
            raise
        try:
            frequency_limits, power_limits = self._get_hardware_limits()
        except Exception:
            self._device.close()
            self._rm.close()
            self._rm = None
            self._device = None
            raise

        if self._reset_on_activate:
            self._command_wait('*RST')
        # Make sure the device starts in a safe state: RF off and no sweep running
        self._device.write(':OUTPut OFF')
        self._command_wait(':SWEep:STATe OFF')

        self._constraints = MicrowaveConstraints(
            power_limits=power_limits,
            frequency_limits=frequency_limits,
            scan_size_limits=(2, self._MAX_STEP_SWEEP_POINTS),
            sample_rate_limits=tuple(self._scan_sample_rate_limits),
            scan_modes=(SamplingOutputMode.JUMP_LIST, SamplingOutputMode.EQUIDISTANT_SWEEP)
        )
        self.log.debug(f'Connected to Siglent {self._model}: frequency limits '
                       f'{frequency_limits} Hz, level limits {power_limits} dBm.')

        self._scan_frequencies = None
        self._scan_power = self._constraints.min_power
        self._scan_mode = SamplingOutputMode.JUMP_LIST
        self._scan_sample_rate = self._constraints.max_sample_rate
        self._in_cw_mode = True

    def on_deactivate(self):
        """ Cleanup performed during deactivation of the module. """
        try:
            self.off()
        finally:
            self._device.close()
            self._rm.close()
            self._device = None
            self._rm = None

    @property
    def constraints(self):
        return self._constraints

    @property
    def is_scanning(self):
        """Read-Only boolean flag indicating if a scan is running at the moment. Can be used
        together with module_state() to determine if the currently running microwave output is a
        scan or CW.
        Should return False if module_state() is 'idle'.

        @return bool: Flag indicating if a scan is running (True) or not (False)
        """
        with self._thread_lock:
            return (self.module_state() != 'idle') and not self._in_cw_mode

    @property
    def cw_power(self):
        """Read-only property returning the currently configured CW microwave power in dBm.

        @return float: The currently set CW microwave power in dBm.
        """
        with self._thread_lock:
            return float(self._device.query(':POWer?'))

    @property
    def cw_frequency(self):
        """Read-only property returning the currently set CW microwave frequency in Hz.

        @return float: The currently set CW microwave frequency in Hz.
        """
        with self._thread_lock:
            return float(self._device.query(':FREQuency?'))

    @property
    def scan_power(self):
        """Read-only property returning the currently configured microwave power in dBm used for
        scanning.

        @return float: The currently set scanning microwave power in dBm
        """
        with self._thread_lock:
            return self._scan_power

    @property
    def scan_frequencies(self):
        """Read-only property returning the currently configured microwave frequencies used for
        scanning.

        In case of self.scan_mode == SamplingOutputMode.JUMP_LIST, this will be a 1D numpy array.
        In case of self.scan_mode == SamplingOutputMode.EQUIDISTANT_SWEEP, this will be a tuple
        containing 3 values (freq_begin, freq_end, number_of_samples).
        If no frequency scan has been configured, return None.

        @return float[]: The currently set scanning frequencies. None if not set.
        """
        with self._thread_lock:
            return self._scan_frequencies

    @property
    def scan_mode(self):
        """Read-only property returning the currently configured scan mode Enum.

        @return SamplingOutputMode: The currently set scan mode Enum
        """
        with self._thread_lock:
            return self._scan_mode

    @property
    def scan_sample_rate(self):
        """Read-only property returning the currently configured scan sample rate in Hz.

        @return float: The currently set scan sample rate in Hz
        """
        with self._thread_lock:
            return self._scan_sample_rate

    def off(self):
        """Switches off any microwave output (both scan and CW).
        Must return AFTER the device has actually stopped.
        """
        with self._thread_lock:
            if self.module_state() == 'idle':
                return
            self._rf_off()
            self._command_wait(':SWEep:STATe OFF')
            self.module_state.unlock()

    def set_cw(self, frequency, power):
        """Configure the CW microwave output. Does not start physical signal output, see also
        "cw_on".

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to set CW parameters. Microwave output active.')
            self._assert_cw_parameters_args(frequency, power)
            self._assert_level_in_band(power, frequency, frequency)

            self._device.write(':SWEep:STATe OFF')
            self._device.write(f':FREQuency {frequency:.6f}')
            self._command_wait(f':POWer {power:.2f}')

    def cw_on(self):
        """ Switches on preconfigured cw microwave output, see also "set_cw".

        Must return AFTER the output is actually active.
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                if self._in_cw_mode:
                    return
                raise RuntimeError(
                    'Unable to start CW microwave output. Frequency scanning in progress.'
                )

            self._command_wait(':SWEep:STATe OFF')
            self._in_cw_mode = True
            self._rf_on()
            self.module_state.lock()

    def configure_scan(self, power, frequencies, mode, sample_rate):
        """Configure a frequency scan.

        @param float power: the power in dBm to be used during the scan
        @param float[] frequencies: an array of all frequencies (jump list)
                                    or a tuple of start, stop frequency and number of steps
                                    (equidistant sweep)
        @param SamplingOutputMode mode: enum stating the way how the frequencies are defined
        @param float sample_rate: external scan trigger rate
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                raise RuntimeError('Unable to configure frequency scan. Microwave output active.')
            self._assert_scan_configuration_args(power, frequencies, mode, sample_rate)
            if mode == SamplingOutputMode.JUMP_LIST:
                if len(frequencies) > self._MAX_LIST_SWEEP_POINTS:
                    raise ValueError(
                        f'JUMP_LIST scan with {len(frequencies):d} points exceeds the '
                        f'{self._MAX_LIST_SWEEP_POINTS:d} point list sweep limit of the '
                        f'SSG5000X. Use fewer points/oversampling or EQUIDISTANT_SWEEP mode.'
                    )
                self._assert_level_in_band(power, min(frequencies), max(frequencies))
            else:
                self._assert_level_in_band(power, *sorted(frequencies[:2]))

            self._device.write(':SWEep:STATe OFF')
            # With STATe FREQuency only the frequency is swept and the level stays at :POWer
            self._device.write(f':POWer {power:.2f}')
            if mode == SamplingOutputMode.EQUIDISTANT_SWEEP:
                self._write_step_sweep(*frequencies)
                self._scan_frequencies = (float(frequencies[0]),
                                          float(frequencies[1]),
                                          int(frequencies[2]))
            else:
                frequencies = np.asarray(frequencies, dtype=np.float64)
                self._write_list_sweep(frequencies, power)
                self._scan_frequencies = frequencies
            self._configure_sweep_trigger()

            self._scan_power = power
            self._scan_mode = mode
            self._scan_sample_rate = sample_rate

    def start_scan(self):
        """Switches on the preconfigured microwave scanning, see also "configure_scan".

        Must return AFTER the output is actually active (and can receive triggers for example).
        """
        with self._thread_lock:
            if self.module_state() != 'idle':
                if not self._in_cw_mode:
                    return
                raise RuntimeError('Unable to start frequency scan. CW microwave output is active.')
            if self._scan_frequencies is None:
                raise RuntimeError('No scan_frequencies set. Unable to start scan.')

            self._command_wait(':SWEep:STATe FREQuency')
            self._in_cw_mode = False
            self._rf_on()
            self.module_state.lock()

    def reset_scan(self):
        """Reset currently running scan and return to start frequency.
        The SSG5000X has no dedicated sweep reset command, so the sweep is re-armed by switching
        the sweep off and on again while leaving the RF output enabled.
        """
        with self._thread_lock:
            if self.module_state() == 'idle':
                return
            if self._in_cw_mode:
                raise RuntimeError('Can not reset frequency scan. CW microwave output active.')

            self._device.write(':SWEep:STATe OFF')
            self._command_wait(':SWEep:STATe FREQuency')

    def _get_hardware_limits(self):
        """ Derives frequency and level limits from the connected model (see module docstring),
        narrowed by the optional config options.

        @return tuple: ((min_frequency, max_frequency), (min_power, max_power))
        """
        series = self._model.upper().split('-')[0]
        if series not in self._MODEL_MAX_FREQUENCY:
            raise RuntimeError(f'Unsupported model "{self._model}". Supported Siglent models are '
                               f'{", ".join(self._MODEL_MAX_FREQUENCY)} (and -V variants).')
        frequency_limits = (self._MIN_FREQUENCY, self._MODEL_MAX_FREQUENCY[series])
        power_limits = self._level_range(*frequency_limits)
        if self._frequency_limits is not None:
            frequency_limits = self._narrow_limits(frequency_limits,
                                                   self._frequency_limits,
                                                   'frequency_limits')
            power_limits = self._level_range(*frequency_limits)
        if self._power_limits is not None:
            power_limits = self._narrow_limits(power_limits, self._power_limits, 'power_limits')
        return frequency_limits, power_limits

    @staticmethod
    def _narrow_limits(hw_limits, cfg_limits, name):
        low, high = max(hw_limits[0], min(cfg_limits)), min(hw_limits[1], max(cfg_limits))
        if low >= high:
            raise ValueError(f'Config option "{name}" {tuple(cfg_limits)} does not overlap with '
                             f'the hardware limits {hw_limits}.')
        return float(low), float(high)

    def _level_range(self, min_frequency, max_frequency):
        """ Widest level setting range over all frequency bands touched by
        [min_frequency, max_frequency].
        """
        bands = self._bands_in_range(min_frequency, max_frequency)
        return min(band[1] for band in bands), max(band[2] for band in bands)

    def _bands_in_range(self, min_frequency, max_frequency):
        band_ends = [band[0] for band in self._LEVEL_SETTING_RANGES[1:]] + [np.inf]
        return [band for band, end in zip(self._LEVEL_SETTING_RANGES, band_ends)
                if band[0] <= max_frequency and end > min_frequency]

    def _assert_level_in_band(self, power, min_frequency, max_frequency):
        """ Checks the level against the datasheet level setting range of every frequency band
        touched by [min_frequency, max_frequency].
        """
        for start, low, high in self._bands_in_range(min_frequency, max_frequency):
            if not low <= power <= high:
                raise ValueError(f'Level {power} dBm is outside the SSG5000X level setting range '
                                 f'[{low}, {high}] dBm for frequencies from {start:.3e} Hz.')

    def _write_step_sweep(self, start, stop, points):
        """ Configures the equidistant step sweep. Caller must hold the thread lock. """
        self._device.write(':SWEep:TYPE STEP')
        self._device.write(':SWEep:STEP:SPACe LINear')
        self._device.write(':SWEep:STEP:SHAPe SAWTooth')
        self._device.write(':SWEep:DIRect FWD')
        self._device.write(f':SWEep:STEP:STARt:FREQuency {start:.6f}')
        self._device.write(f':SWEep:STEP:STOP:FREQuency {stop:.6f}')
        self._device.write(f':SWEep:STEP:POINts {int(points):d}')
        self._command_wait(f':SWEep:STEP:DWELl {self._MIN_DWELL_TIME:.3f}')

    def _write_list_sweep(self, frequencies, power):
        """ Replaces the instrument sweep list with the given frequencies. Caller must hold the
        thread lock.
        """
        self._device.write(':SWEep:TYPE LIST')
        self._device.write(':SWEep:DIRect FWD')
        self._command_wait(':SWEep:LIST:INITialize:PRESet')
        # Depending on firmware, a cleared list may still contain a single default row
        stale_rows = int(self._device.query(':SWEep:LIST:CPOint?'))
        for freq in frequencies:
            self._device.write(
                f':SWEep:LIST:ADDList {freq:.6f},{power:.2f},{self._MIN_DWELL_TIME:.3f}'
            )
        for _ in range(stale_rows):
            self._device.write(':SWEep:LIST:DELete 1')
        self._device.query('*OPC?')

        written_rows = int(self._device.query(':SWEep:LIST:CPOint?'))
        if written_rows != len(frequencies):
            raise RuntimeError(f'Sweep list upload failed. Expected {len(frequencies):d} rows '
                               f'on device but found {written_rows:d}.')

    def _configure_sweep_trigger(self):
        """ Arm the whole sweep immediately and step each point on an external trigger. Caller
        must hold the thread lock.
        """
        slope = 'POSitive' if self._trigger_edge == 'rising' else 'NEGative'
        self._device.write(':SWEep:MODE CONTinue')
        self._device.write(':SWEep:SWEep:TRIGger:TYPE AUTO')
        self._device.write(':SWEep:POINt:TRIGger:TYPE EXT')
        self._command_wait(f':INPut:TRIGger:SLOPe {slope}')

    def _command_wait(self, command_str):
        """ Writes the command in command_str via PyVisa and waits until the device has finished
        processing it.

        @param str command_str: The command to be written
        """
        self._device.write(command_str)
        self._device.query('*OPC?')

    def _rf_on(self):
        """ Switches on any preconfigured microwave output. """
        self._device.write(':OUTPut ON')
        while not self._output_active():
            time.sleep(0.1)

    def _rf_off(self):
        """ Switches off the microwave output. """
        self._device.write(':OUTPut OFF')
        while self._output_active():
            time.sleep(0.1)

    def _output_active(self):
        return bool(int(float(self._device.query(':OUTPut?').strip())))
