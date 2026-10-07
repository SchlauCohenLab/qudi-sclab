# -*- coding: utf-8 -*-

"""
This file contains the qudi hardware module to use a National Instruments R-series FPGA
target — a standalone Multifunction RIO card (e.g. PXI-7852R) or a CompactRIO chassis
(cRIO controller + C-Series modules) — as a mixed signal input data streamer, most commonly
for APD/SPCM photon counting.

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
Unlike NI X-series boards, NI R-series FPGA targets — standalone Multifunction RIO cards
(PCI/PXI-781x/783x/784x/785x, including the PXI-7852R) as well as CompactRIO chassis, which
embed the same kind of user-programmable FPGA — are NOT supported by NI-DAQmx. They only talk
to the host through the NI-RIO driver stack and a user-compiled LabVIEW FPGA bitfile (.lvbitx).
There is no generic "DAQmx personality" for these targets, so this module can not simply reuse
nidaqmx tasks like "ni_x_series_instream.py" does. Instead it drives the target through the
"nifpga" Python package (https://github.com/ni/nifpga-python), which talks to a Session opened
against your compiled bitfile and its exposed registers/FIFOs. `resource_name` is whatever
resource string NI MAX (or your LabVIEW FPGA project) shows for the target, e.g. "RIO0" for a
local card, or the assigned name/IP of a networked CompactRIO controller.

Because the whole point of an "R" series target is that its I/O personality is user-defined,
this module can only talk to a bitfile that exposes the specific FPGA<->host interface
documented below. You (or whoever authors the LabVIEW FPGA VI) must implement:

  * One boolean CONTROL that gates the acquisition loop on the FPGA (configurable via
    `acquisition_control_register`). The host sets this True on start_stream() and False on
    stop_stream().
  * One U32 (or compatible) CONTROL that sets the counting window length (configurable via
    `sample_clock_register`), in whichever unit your bitfile actually reads as an input —
    set `sample_clock_units` to 'us' (default) to write round(1e6 / sample_rate), or to 'ticks'
    to write round(fpga_clock_frequency / sample_rate * sample_clock_divider_scale). IMPORTANT:
    make sure the register you point at is actually a CONTROL your block diagram reads from, not
    an INDICATOR the block diagram only writes to for readback/debug — writing to an indicator
    register "succeeds" from the host's point of view but has zero effect on the FPGA logic,
    which silently produces a starved/empty FIFO. An .lvbitx's XML metadata tells you which is
    which per register (`<Indicator>true</Indicator>` vs `false`) even without opening LabVIEW —
    see the verified examples below, where exactly this distinction was the actual bug behind an
    early FifoTimeoutError.
  * Optionally, a second U32 (or compatible) CONTROL for a fixed per-cycle dead/settling time
    (configurable via `stabilization_period_us_register`, default 'Stabilization period [us]',
    set to None to disable) written once from `stabilization_period_us` (default 0.0) every
    start_stream(). When set, the counting window is computed as
    round(1e6/sample_rate - stabilization_period_us) so the full cycle (stabilization + count)
    still matches the requested sample rate.
  * One target-to-host DMA FIFO per configured channel:
      - For each entry in `digital_sources`, a FIFO of unsigned integers that, once per sample
        clock tick, delivers the number of rising edges counted on that digital input terminal
        during the previous sample period (i.e. an edge counter that free-runs and is
        latched+reset every sample clock tick).
      - For each entry in `analog_sources`, a FIFO of signed integers carrying the raw ADC code
        of that AI channel, once per sample clock tick.

  All FIFO and register names are configurable so this module can be pointed at whatever names
  you gave them in your bitfile; the defaults now match the verified working example below.

-----------------------------------------------------------------------------------------------
VERIFIED WORKING EXAMPLE: "SPCM_Main_v4.lvbitx" compiled for the PXI-7852R
-----------------------------------------------------------------------------------------------
Inspecting two real compiled bitfiles for the same design (an .lvbitx is itself XML, so its
RegisterList/Channel definitions can be read directly with e.g. Python's xml.etree, without
needing LabVIEW) shows exactly how easy it is to point this module at the wrong register. An
earlier "SPCM_Main.lvbitx" exposed a "trigger_half_period_ticks" (U32) control alongside a
"Trigger period [us]" (U32) control with no Indicator flag distinguishing them — this module
originally defaulted to writing the raw ticks register, which produced a permanently empty FIFO
(FifoTimeoutError with elements_remaining always 0) because that register turned out not to be
read by the FPGA logic at all. The newer "SPCM_Main_v4.lvbitx" (TargetClass: PXI-7852R, 40 MHz
onboard clock) makes the distinction explicit and unambiguous — the metadata itself marks the
low-level registers as read-only indicators:

  * Register "Gate" (bool, Indicator=false) — start/stop control, matches
    `acquisition_control_register`.
  * Register "Count time [us]" (U32, Indicator=false) — the ACTUAL counting window control, in
    microseconds, matches `sample_clock_register` with `sample_clock_units: 'us'`.
  * Register "Stabilization period [us]" (U32, Indicator=false) — a settling delay before each
    counting window, matches `stabilization_period_us_register`.
  * Registers "half_period_ticks", "period_ticks", "count_time_ticks" (U32, Indicator=true) —
    read-only, FPGA-computed diagnostics mirroring the above in ticks. Do NOT point
    `sample_clock_register` at any of these — writing to them has no effect.
  * DMA Channel "PhotonCount" (U32, target-to-host, depth 1023) — raw counts per window, matches
    a `digital_sources` FIFO entry.

If you're pointed at a different/future revision of this bitfile, always check each candidate
register's `<Indicator>` value in the .lvbitx XML before wiring it into `sample_clock_register` —
Indicator=true means read-only, Indicator=false means it's safe to write.

-----------------------------------------------------------------------------------------------
ADDING APD/SPCM PHOTON COUNTING TO AN EXISTING FPGA VI (e.g. a "FPGA Main.vi" that already
drives a galvo/piezo scanner from a different parallel loop in the same bitfile)
-----------------------------------------------------------------------------------------------
It is common to reuse the FPGA bitfile that already drives your scan stage: LabVIEW FPGA VIs
run every top-level While Loop on the block diagram in parallel, so an APD counting loop can be
added as a new loop alongside an existing scan-position loop, compiled into the same bitfile,
without disturbing what already works. If your APD's TTL pulse line is currently only read as a
plain boolean once per (slow) scan-update loop tick, that is NOT sufficient for photon counting
— SPCM pulses are tens of ns wide and will mostly be missed at scan-loop rates. You need a
dedicated counting loop, structured like this:

  1. A free-running edge counter, in its own Single-Cycle Timed Loop (SCTL) clocked at the
     FPGA's fastest available clock domain (commonly 40 MHz): each iteration, compare the
     current and previous value of the APD digital input line; on a 0->1 transition, increment
     a U32 accumulator held in a feedback node (or a Memory Item shared with the readout loop).
     A 40 MHz sampling rate resolves edges down to 25 ns, which comfortably covers a typical
     SPCM's ~20-50 ns dead time.
  2. A second loop, paced by the same "Loop Timer"/divider mechanism you likely already use for
     the scan-position loop (reading `SampleClockDivider` off a host control), that once per
     window: reads the current accumulator value, pushes it into a new target-to-host DMA FIFO
     (e.g. name it "ApdCountsFifo"), and resets the accumulator to 0. Guard the read-and-reset
     against the counting loop writing concurrently (e.g. an Action Engine / single-element
     FIFO acting as a lock, or a First-In-First-Out "count transfer" FIFO written by loop 1 and
     drained by loop 2) so no edges are lost or double-counted across the boundary.
  3. Gate both loops with the same `AcquisitionActive` boolean control this module writes on
     start_stream()/stop_stream(), so the FPGA doesn't burn DMA bandwidth while idle.

Once that FIFO exists and is named in `digital_sources` below, this module will read it as
counts-per-window and scale by the sample rate to counts/s, exactly like the CI-period counting
trick used on X-series cards — except implemented directly and simply in FPGA gateware.

Example config for copy-paste, matching the verified SPCM_Main_v4.lvbitx interface above:

nicard_apd_instreamer:
    module.Class: 'ni_card.ni_r_series_instream.NIRSeriesFPGAInStreamer'
    options:
        resource_name: 'RIO0'  # as shown for your PXI-7852R in NI MAX
        bitfile_path: 'C:/qudi_fpga_bitfiles/SPCM_Main_v4.lvbitx'  # the compiled bitfile
        digital_sources:  # channel name -> target-to-host FIFO name
            'apd': 'PhotonCount'
        fpga_clock_frequency: 40e6  # confirmed 40 MHz onboard clock
        sample_clock_register: 'Count time [us]'  # default, shown for clarity
        sample_clock_units: 'us'  # default, shown for clarity
        stabilization_period_us_register: 'Stabilization period [us]'  # default, shown for clarity
        stabilization_period_us: 0.0  # optional, added on top of the requested sample period
        acquisition_control_register: 'Gate'  # default, shown for clarity
        min_sample_rate: 1.0  # optional
        max_sample_rate: 1e5  # optional, keep below your SPCM's max count rate
        max_channel_samples_buffer: 10000000  # optional
        read_write_timeout: 10  # optional
        reset_on_activate: True  # optional, set False if the bitfile is shared with
                                 # ni_r_series_pulser so the pulse memory is not wiped

Example config for copy-paste (mixed digital + analog channels, custom bitfile using a raw-ticks
register instead of a microsecond one — override every name explicitly):

nicard_7852_instreamer:
    module.Class: 'ni_card.ni_r_series_instream.NIRSeriesFPGAInStreamer'
    options:
        resource_name: 'RIO0'
        bitfile_path: 'C:/qudi_fpga_bitfiles/NiRSeriesInStream.lvbitx'
        digital_sources:  # optional; channel name -> target-to-host FIFO name
            'acceptor': 'CounterFifo0'
            'donor': 'CounterFifo1'
        analog_sources:  # optional; channel name -> target-to-host FIFO name
            'x': 'AnalogFifo0'
            'y': 'AnalogFifo1'
        adc_voltage_range: [-10, 10]  # optional, fixed ±10V input range on the 7852R
        adc_resolution_bits: 16  # optional, 16-bit ADC on the 7852R
        fpga_clock_frequency: 40e6  # optional, onboard FPGA clock used to derive the sample clock
        sample_clock_register: 'CounterWindowTicks'  # must match your bitfile
        sample_clock_units: 'ticks'  # this bitfile's register holds raw ticks, not microseconds
        stabilization_period_us_register: null  # disable, this bitfile has no such register
        acquisition_control_register: 'AcquisitionActive'  # must match your bitfile
        min_sample_rate: 1.0  # optional
        max_sample_rate: 750e3  # optional, 750 kS/s per dedicated ADC on the 7852R
        max_channel_samples_buffer: 10000000  # optional
        read_write_timeout: 10  # optional

"""

import time
import numpy as np
import nifpga
from typing import Tuple, List, Optional, Sequence, Union

from qudi.core.configoption import ConfigOption
from qudi.util.constraints import ScalarConstraint
from qudi.interface.data_instream_interface import DataInStreamInterface, DataInStreamConstraints
from qudi.interface.detector_interface import DetectorInterface, Channel, SampleTiming, StreamingMode


class _FpgaChannelReader:
    """ Helper wrapping a single target-to-host FIFO together with the scaling needed to turn
    its raw integer payload into the physical unit of the channel it belongs to.
    """

    def __init__(self, fifo, kind, scale=1.0, offset=0.0):
        self.fifo = fifo
        self.kind = kind  # 'digital' (counts/s) or 'analog' (V)
        self.scale = scale
        self.offset = offset
        self.started = False

    def start(self, requested_depth):
        self.fifo.configure(requested_depth)
        self.fifo.start()
        self.started = True

    def stop(self):
        if self.started:
            self.fifo.stop()
            self.started = False

    def available_samples(self):
        return self.fifo.read(number_of_elements=0, timeout_ms=0).elements_remaining

    def _to_physical(self, raw, sample_rate):
        data = np.asarray(raw, dtype=np.float64)
        if self.kind == 'digital':
            data *= sample_rate
        else:
            data = data * self.scale + self.offset
        return data

    def read_many(self, number_of_samples, timeout, sample_rate):
        result = self.fifo.read(number_of_elements=number_of_samples,
                                timeout_ms=int(timeout * 1000))
        return self._to_physical(result.data, sample_rate)

    def read_one(self, timeout, sample_rate):
        result = self.fifo.read(number_of_elements=1, timeout_ms=int(timeout * 1000))
        return self._to_physical(result.data, sample_rate)[0]


class NIRSeriesFPGAInStreamer(DataInStreamInterface, DetectorInterface):
    """
    A National Instruments R-series FPGA target — a standalone Multifunction RIO card
    (e.g. PXI-7852R) or a CompactRIO chassis — driven through a user-supplied LabVIEW FPGA
    bitfile, that can count digital pulses (e.g. APD/SPCM photon counting) and measure analog
    voltages as a data stream.

    !!!!!! NI R-series FPGA targets (standalone PCI/PXI-781x/783x/784x/785x cards, or a
    !!!!!! CompactRIO chassis) ONLY !!!!!!
    !!!!!! Requires a compiled bitfile implementing the FPGA<->host interface documented
    !!!!!! at the top of this file. !!!!!!

    See the module docstring for the required FPGA register/FIFO interface, guidance on adding
    an APD counting loop to an existing bitfile, and example configs.
    """

    # config options
    _resource_name = ConfigOption(name='resource_name', default='RIO0', missing='warn')
    _bitfile_path = ConfigOption(name='bitfile_path', default=None, missing='error')
    _digital_sources = ConfigOption(name='digital_sources', default=dict(), missing='info')
    _analog_sources = ConfigOption(name='analog_sources', default=dict(), missing='info')
    _adc_voltage_range = ConfigOption('adc_voltage_range', default=(-10, 10), missing='info')
    _adc_resolution_bits = ConfigOption('adc_resolution_bits', default=16, missing='nothing')
    _fpga_clock_frequency = ConfigOption('fpga_clock_frequency',
                                         default=40e6,
                                         missing='nothing',
                                         constructor=lambda x: float(x))
    _sample_clock_register = ConfigOption('sample_clock_register',
                                          default='Count time [us]',
                                          missing='nothing')
    _sample_clock_units = ConfigOption('sample_clock_units',
                                       default='us',
                                       missing='nothing',
                                       constructor=lambda x: str(x).lower())
    _sample_clock_divider_scale = ConfigOption('sample_clock_divider_scale',
                                               default=1.0,
                                               missing='nothing',
                                               constructor=lambda x: float(x))
    _stabilization_period_us_register = ConfigOption('stabilization_period_us_register',
                                                      default='Stabilization period [us]',
                                                      missing='nothing')
    _stabilization_period_us = ConfigOption('stabilization_period_us',
                                            default=0.0,
                                            missing='nothing',
                                            constructor=lambda x: float(x))
    _acquisition_control_register = ConfigOption('acquisition_control_register',
                                                  default='Gate',
                                                  missing='nothing')
    _min_sample_rate = ConfigOption('min_sample_rate', default=1.0, missing='nothing')
    _max_sample_rate = ConfigOption('max_sample_rate', default=750e3, missing='nothing')
    _max_channel_samples_buffer = ConfigOption(name='max_channel_samples_buffer',
                                               default=1024**2,
                                               missing='info',
                                               constructor=lambda x: max(int(round(x)), 1024**2))
    _rw_timeout = ConfigOption('read_write_timeout', default=10, missing='nothing')
    # Set to False when the bitfile is shared with another module (e.g. ni_r_series_pulser)
    _reset_on_activate = ConfigOption('reset_on_activate', default=True, missing='nothing')

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # nifpga Session handle
        self._session = None
        # channel name -> _FpgaChannelReader, for every configured (not necessarily active) channel
        self._channel_readers = dict()
        # Internal settings
        self.__sample_rate = -1
        self.__buffer_size = -1
        self.__streaming_mode = None
        # currently active channels
        self.__active_channels = tuple()
        # Stored hardware constraints
        self._constraints = None
        self._channels = dict()

    def on_activate(self):
        """
        Downloads the bitfile to the FPGA and performs sanity checks.
        """
        if not self._bitfile_path:
            raise ValueError('"bitfile_path" must be specified in config. It must point to a '
                             'compiled LabVIEW FPGA bitfile (.lvbitx) implementing the '
                             'FPGA<->host interface documented in ni_r_series_instream.py.')

        try:
            self._session = nifpga.Session(bitfile=self._bitfile_path, resource=self._resource_name)
            # Some bitfiles (e.g. compiled with "Configure how VI runs" -> auto-run disabled;
            # this shows up as <AutoRunWhenDownloaded>false</AutoRunWhenDownloaded> in the
            # .lvbitx XML) only get downloaded onto the FPGA when the Session opens, but their
            # loops do NOT start executing on their own. Without an explicit run() here, every
            # register write still "succeeds" but has no effect (nothing is reading it), and
            # every FIFO read blocks for the full timeout with zero elements ever produced.
            # reset() + run() is a harmless no-op if the bitfile auto-runs on download anyway.
            # Skip the reset if another module shares this bitfile, it would wipe its FPGA state.
            if self._reset_on_activate:
                self._session.reset()
            self._session.run()
        except Exception as err:
            raise RuntimeError(
                f'Could not open nifpga Session for resource "{self._resource_name}" with '
                f'bitfile "{self._bitfile_path}". Is the FPGA card connected and the resource '
                f'name correct?'
            ) from err

        # Check required control registers exist in the bitfile
        required_registers = [self._sample_clock_register, self._acquisition_control_register]
        if self._stabilization_period_us_register is not None:
            required_registers.append(self._stabilization_period_us_register)
        for reg_name in required_registers:
            if reg_name not in self._session.registers:
                self._terminate_session()
                raise ValueError(
                    f'Control register "{reg_name}" not found in bitfile. Available registers: '
                    f'{list(self._session.registers)}'
                )

        # Check digital input FIFOs
        if self._digital_sources:
            invalid = {ch: fifo for ch, fifo in self._digital_sources.items()
                      if fifo not in self._session.fifos}
            if invalid:
                self.log.error(
                    'Invalid digital source FIFOs encountered. Following channels will be '
                    'ignored:\n  {0}\nValid FIFOs in bitfile are:\n  {1}'
                    ''.format(', '.join(invalid), ', '.join(self._session.fifos)))
            for ch in invalid:
                self._digital_sources.pop(ch)

        # Check analog input FIFOs
        if self._analog_sources:
            invalid = {ch: fifo for ch, fifo in self._analog_sources.items()
                      if fifo not in self._session.fifos}
            if invalid:
                self.log.error(
                    'Invalid analog source FIFOs encountered. Following channels will be '
                    'ignored:\n  {0}\nValid FIFOs in bitfile are:\n  {1}'
                    ''.format(', '.join(invalid), ', '.join(self._session.fifos)))
            for ch in invalid:
                self._analog_sources.pop(ch)

        if not self._analog_sources and not self._digital_sources:
            raise ValueError(
                'No valid analog or digital sources defined in config. Activation of '
                'NIRSeriesFPGAInStreamer failed!'
            )

        # Build analog code -> volts scaling
        min_v, max_v = min(self._adc_voltage_range), max(self._adc_voltage_range)
        span = max_v - min_v
        code_scale = span / (2 ** self._adc_resolution_bits)
        code_offset = (max_v + min_v) / 2

        # Create constraints
        channel_units = {chnl: 'counts/s' for chnl in self._digital_sources.keys()}
        channel_units.update({chnl: 'V' for chnl in self._analog_sources.keys()})
        sample_rate_constraint = ScalarConstraint(default=min(50.0, self._max_sample_rate),
                                                  bounds=(self._min_sample_rate,
                                                          self._max_sample_rate),
                                                  increment=1,
                                                  enforce_int=False)
        buffer_size_constraint = ScalarConstraint(default=1024**2,
                                                  bounds=(2, self._max_channel_samples_buffer),
                                                  increment=1,
                                                  enforce_int=True)
        self._constraints = DataInStreamConstraints(
            channel_units=channel_units,
            sample_timing=SampleTiming.CONSTANT,
            streaming_modes=[StreamingMode.CONTINUOUS],  # TODO: Implement FINITE streaming mode
            data_type=np.float64,
            channel_buffer_size=buffer_size_constraint,
            sample_rate=sample_rate_constraint
        )

        self._channels = dict()
        for ch in channel_units:
            self._channels[ch] = Channel(ch,
                                         unit=channel_units[ch],
                                         dtype=np.float64,
                                         streaming_mode=StreamingMode.CONTINUOUS,
                                         buffer_size=buffer_size_constraint,
                                         sample_timing=SampleTiming.CONSTANT,
                                         sample_rate=sample_rate_constraint,
                                         bin_width=None)

        # Build the channel readers (FIFOs stay stopped until start_stream())
        self._channel_readers = dict()
        for ch, fifo_name in self._digital_sources.items():
            self._channel_readers[ch] = _FpgaChannelReader(self._session.fifos[fifo_name],
                                                            kind='digital')
        for ch, fifo_name in self._analog_sources.items():
            self._channel_readers[ch] = _FpgaChannelReader(self._session.fifos[fifo_name],
                                                            kind='analog',
                                                            scale=code_scale,
                                                            offset=code_offset)

        self._session.registers[self._acquisition_control_register].write(False)

        self.configure(active_channels=self._constraints.channel_units,
                       streaming_mode=StreamingMode.CONTINUOUS,
                       channel_buffer_size=self._constraints.channel_buffer_size.default,
                       sample_rate=self._constraints.sample_rate.default)

    def on_deactivate(self):
        """ Shut down the FPGA session. """
        self._terminate_session()

    @property
    def constraints(self) -> DataInStreamConstraints:
        """ Read-only property returning the constraints on the settings for this data streamer. """
        return self._constraints

    @property
    def sample_rate(self):
        """ Read-only property returning the currently set sample rate in Hz. """
        return self.__sample_rate

    @property
    def channel_buffer_size(self) -> int:
        """ Read-only property returning the currently set buffer size in samples per channel. """
        return self.__buffer_size

    @property
    def streaming_mode(self) -> StreamingMode:
        """ Read-only property returning the currently configured StreamingMode Enum """
        return self.__streaming_mode

    @property
    def active_channels(self) -> List[str]:
        """ Read-only property returning the currently configured active channel names """
        return list(self.__active_channels)

    def configure(self,
                  active_channels: Sequence[str],
                  streaming_mode: Union[StreamingMode, int],
                  channel_buffer_size: int,
                  sample_rate: float) -> None:
        """ Configure a data stream. See read-only properties for information on each parameter. """
        if self.module_state() == 'locked':
            raise RuntimeError('Unable to configure data stream while it is already running')
        if isinstance(streaming_mode, int):
            streaming_mode = StreamingMode(streaming_mode)
        channel_buffer_size = int(round(channel_buffer_size))
        if any(ch not in self._constraints.channel_units for ch in active_channels):
            raise ValueError(
                f'Invalid channel to stream from encountered {tuple(active_channels)}. \n'
                f'Valid channels are: {tuple(self._constraints.channel_units)}'
            )
        if streaming_mode not in self._constraints.streaming_modes or streaming_mode == StreamingMode.INVALID:
            raise ValueError(f'Invalid streaming mode "{streaming_mode}" encountered.\n'
                             f'Valid modes are: {self._constraints.streaming_modes}.')
        self._constraints.channel_buffer_size.check(channel_buffer_size)
        self._constraints.sample_rate.check(sample_rate)

        self.__active_channels = tuple(active_channels)
        self.__streaming_mode = streaming_mode
        self.__buffer_size = channel_buffer_size
        self.__sample_rate = sample_rate

    @property
    def available_samples(self):
        """ Read-only property to return the currently available number of samples per channel ready
        to read from buffer.
        """
        if self.module_state() == 'locked' and self.__active_channels:
            return min(self._channel_readers[ch].available_samples()
                      for ch in self.__active_channels)
        else:
            return 0

    def start_stream(self) -> None:
        """ Start the data acquisition/streaming """
        if self.module_state() == 'locked':
            self.log.warning('Unable to start input stream. It is already running.')
        else:
            self.module_state.lock()
            try:
                if self._stabilization_period_us_register is not None:
                    stabilization_us = max(0, int(round(self._stabilization_period_us)))
                    self._session.registers[self._stabilization_period_us_register].write(
                        stabilization_us
                    )
                else:
                    stabilization_us = 0

                if self._sample_clock_units == 'ticks':
                    value = max(1, int(round(
                        self._fpga_clock_frequency / self.__sample_rate
                        * self._sample_clock_divider_scale
                    )))
                else:
                    count_time_us = 1e6 / self.__sample_rate - stabilization_us
                    if count_time_us <= 0:
                        raise ValueError(
                            f'Requested sample_rate {self.__sample_rate:.3e} Hz is too fast for '
                            f'the configured stabilization_period_us ({stabilization_us} us) to '
                            f'leave any positive counting window. Lower the sample rate or '
                            f'stabilization_period_us.'
                        )
                    value = max(1, int(round(count_time_us)))
                self._session.registers[self._sample_clock_register].write(value)
                for ch in self.__active_channels:
                    self._channel_readers[ch].start(self.__buffer_size)
                self._session.registers[self._acquisition_control_register].write(True)
            except:
                self.module_state.unlock()
                self._stop_all_readers()
                raise

    def stop_stream(self) -> None:
        """ Stop the data acquisition/streaming """
        try:
            self._session.registers[self._acquisition_control_register].write(False)
        except Exception:
            self.log.exception('Error while disabling FPGA acquisition loop.')
        finally:
            self._stop_all_readers()
            if self.module_state() == 'locked':
                self.module_state.unlock()

    def read_data_into_buffer(self,
                              data_buffer: np.ndarray,
                              samples_per_channel: int = None,
                              timestamp_buffer: Optional[np.ndarray] = None) -> None:
        """ Read data from the stream buffer into a 1D numpy array given as parameter.
        Samples of all channels are stored interleaved in contiguous memory.
        In case of a multidimensional buffer array, this buffer will be flattened before written
        into.
        The 1D data_buffer can be unraveled into channel and sample indexing with:

            data_buffer.reshape([<samples_per_channel>, <channel_count>])

        The data_buffer array must have the same data type as self.constraints.data_type.

        This function is blocking until the required number of samples has been acquired.
        """
        if self.module_state() != 'locked':
            raise RuntimeError('Unable to read data. Device is not running.')
        # Check for buffer overflow
        if self.available_samples > self.__buffer_size:
            raise OverflowError('Hardware channel buffer has overflown. Please increase readout '
                                'speed or decrease sample rate.')
        if not isinstance(data_buffer, np.ndarray) or data_buffer.dtype != self._constraints.data_type:
            raise TypeError(
                f'data_buffer must be numpy.ndarray with dtype {self._constraints.data_type}'
            )

        channel_count = len(self.__active_channels)
        if samples_per_channel is None:
            samples_per_channel = len(data_buffer) // channel_count
        total_samples = channel_count * samples_per_channel
        if samples_per_channel > 0:
            try:
                for i, ch in enumerate(self.__active_channels):
                    data_buffer[i:total_samples:channel_count] = self._channel_readers[ch].read_many(
                        samples_per_channel, self._rw_timeout, self.__sample_rate
                    )[:samples_per_channel]
            except:
                self.log.exception('Getting samples from streamer failed. Stopping streamer.')
                self.stop_stream()

    def read_available_data_into_buffer(self,
                                        data_buffer: np.ndarray,
                                        timestamp_buffer: Optional[np.ndarray] = None) -> int:
        """ Read data from the stream buffer into a 1D numpy array given as parameter.
        All samples for each channel are stored in consecutive blocks one after the other.
        The number of samples read per channel is returned and can be used to slice out valid data
        from the buffer arrays like:

            valid_data = data_buffer[:<channel_count> * <return_value>]

        See "read_data_into_buffer" documentation for more details.
        """
        channel_count = len(self.__active_channels)
        samples_per_channel = min(self.available_samples, data_buffer.size // channel_count)
        self.read_data_into_buffer(data_buffer=data_buffer,
                                   samples_per_channel=samples_per_channel,
                                   timestamp_buffer=timestamp_buffer)
        return samples_per_channel

    def read_data(self,
                  samples_per_channel: Optional[int] = None
                  ) -> Tuple[np.ndarray, Union[np.ndarray, None]]:
        """ Read data from the stream buffer into a 1D numpy array and return it. """
        if samples_per_channel is None:
            samples_per_channel = self.available_samples
        channel_count = len(self.__active_channels)
        data_buffer = np.empty(samples_per_channel * channel_count,
                               dtype=self._constraints.data_type)
        self.read_data_into_buffer(data_buffer=data_buffer, samples_per_channel=samples_per_channel)
        return data_buffer, None

    def read_single_point(self) -> Tuple[np.ndarray, Union[None, np.float64]]:
        """ This method will initiate a single sample read on each configured data channel.
        In general this sample may not be acquired simultaneous for all channels.
        """
        if self.module_state() != 'locked':
            raise RuntimeError('Unable to read data. Device is not running.')

        data_buffer = np.empty(len(self.__active_channels), dtype=self._constraints.data_type)
        try:
            for i, ch in enumerate(self.__active_channels):
                data_buffer[i] = self._channel_readers[ch].read_one(self._rw_timeout,
                                                                    self.__sample_rate)
        except:
            self.log.exception('Getting samples from data stream failed. Stopping streamer.')
            self.stop_stream()
        return data_buffer, None

    # =============================================================================================

    def get_constraints(self):
        """ Read-only property returning the constraints on the settings for this data streamer. """
        return [ch for ch in self._channels.values()]

    def configure_channels(self, channels_settings):
        """ Configure a data stream. See read-only properties for information on each parameter. """
        pass

    def get_data(self, channels=None):
        """ Polls the current timetrace data from the FPGA counter/ADC channels.

        Return value is a dict of channel name -> single sample.
        """
        if self.module_state() == 'locked':
            data = np.array(self.read_single_point()[0])
        else:
            self.start_stream()
            data = np.array(self.read_single_point()[0])
            self.stop_stream()
        return {ch: data[i] for i, ch in enumerate(self.__active_channels)}

    def get_status(self, channels=None):
        """ Get the status of the channels

        @return dict: with the channel label as key and the status number as item.
        """
        return {ch: 0 for ch in self.__active_channels}

    # =============================================================================================

    def _stop_all_readers(self):
        for reader in self._channel_readers.values():
            try:
                reader.stop()
            except Exception:
                self.log.exception('Error while trying to stop FPGA FIFO.')

    def _terminate_session(self):
        self._stop_all_readers()
        if self._session is not None:
            try:
                self._session.registers[self._acquisition_control_register].write(False)
            except Exception:
                pass
            try:
                self._session.close()
            except Exception:
                self.log.exception('Error while trying to close nifpga Session.')
            finally:
                self._session = None
