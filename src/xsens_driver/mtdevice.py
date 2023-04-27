#!/usr/bin/env python
from __future__ import print_function

import serial
import struct
import sys
import time
import glob

from xsens_driver.mtdef import MID, OutputMode, OutputSettings, MTException, \
    Baudrates, XDIGroup, getMIDName, DeviceState, DeprecatedMID, \
    MTErrorMessage, MTTimeoutException


def _byte(b):
    """Convert the given character or byte to a byte."""
    if sys.version_info[0] == 2:
        return ord(b)
    if isinstance(b, bytearray):
        return b[0]
    return b


################################################################
# MTDevice class
################################################################
class MTDevice(object):
    """XSens MT device communication object."""

    def __init__(self, port, baudrate=115200, timeout=0.002, autoconf=True,
                 config_mode=False, verbose=False, initial_wait=0.1):
        """Open device."""
        self.verbose = verbose
        # serial interface to the device
        try:
            self.device = serial.Serial(port, baudrate, timeout=timeout,
                                        writeTimeout=timeout)
        except IOError:
            # FIXME with pyserial3 we might need some specific flags
            self.device = serial.Serial(port, baudrate, timeout=timeout,
                                        writeTimeout=timeout, rtscts=True,
                                        dsrdtr=True)
        time.sleep(initial_wait)  # open returns before device is ready
        self.device.flushInput()
        self.device.flushOutput()
        # timeout for communication
        self.timeout = 100*timeout
        # state of the device
        self.state = None
        if autoconf:
            self.auto_config_legacy()
        else:
            # mode parameter of the IMU
            self.mode = None
            # settings parameter of the IMU
            self.settings = None
            # length of the MTData message
            self.length = None
            # header of the MTData message
            self.header = None
        if config_mode:
            self.GoToConfig()

    ############################################################
    # Low-level communication
    ############################################################
    def write_msg(self, mid, data=b''):
        """Low-level message sending function."""
        length = len(data)
        if length > 254:
            lendat = b'\xFF' + struct.pack('!H', length)
        else:
            lendat = struct.pack('!B', length)
        packet = b'\xFA\xFF' + struct.pack('!B', mid) + lendat + data
        packet += struct.pack('!B', 0xFF & (-(sum(map(_byte, packet[1:])))))
        msg = packet
        start = time.time()
        while ((time.time()-start) < self.timeout) and self.device.read():
            pass
        try:
            self.device.write(msg)
        except serial.serialutil.SerialTimeoutException:
            raise MTTimeoutException("writing message")
        if self.verbose:
            print("MT: Write message id 0x%02X (%s) with %d data bytes: "
                  "[%s]" % (mid, getMIDName(mid), length,
                            ' '.join("%02X" % _byte(v) for v in data)))

    def waitfor(self, size=1):
        """Get a given amount of data."""
        buf = bytearray()
        for _ in range(100):
            buf.extend(self.device.read(size-len(buf)))
            if len(buf) == size:
                return buf
            if self.verbose:
                print("waiting for %d bytes, got %d so far: [%s]" % 
                    (size, len(buf), ' '.join('%02X' % v for v in buf)))
        raise MTTimeoutException("waiting for message")

    def read_data_msg(self, buf=bytearray()):
        """Low-level MTData receiving function.
        Take advantage of known message length.
        """
        start = time.time()
        if self.length <= 254:
            totlength = 5 + self.length
        else:
            totlength = 7 + self.length
        while (time.time()-start) < self.timeout:
            buf.extend(self.waitfor(totlength - len(buf)))
            preamble_ind = buf.find(self.header)
            if preamble_ind == -1:  # not found
                # discard unexploitable data
                if self.verbose:
                    sys.stderr.write("MT: discarding (no preamble).\n")
                del buf[:-3]
                continue
            elif preamble_ind:  # found but not at start
                # discard leading bytes
                if self.verbose:
                    sys.stderr.write("MT: discarding (before preamble).\n")
                del buf[:preamble_ind]
                # complete message for checksum
                buf.extend(self.waitfor(totlength-len(buf)))
            if 0xFF & sum(buf[1:]):
                if self.verbose:
                    sys.stderr.write("MT: invalid checksum; discarding data "
                                     "and waiting for next message.\n")
                del buf[:buf.find(self.header)-2]
                continue
            data = str(buf[-self.length-1:-1])
            del buf[:]
            return data
        else:
            raise MTException("could not find MTData message.")

    def read_msg(self):
        """Low-level message receiving function."""
        start = time.time()
        while (time.time()-start) < self.timeout:
            # first part of preamble
            if _byte(self.waitfor()) != 0xFA:
                continue
            # second part of preamble
            if _byte(self.waitfor()) != 0xFF:  # we assume no timeout anymore
                continue
            # read message id and length of message
            mid, length = struct.unpack('!BB', self.waitfor(2))
            if length == 255:    # extended length
                length, = struct.unpack('!H', self.waitfor(2))
            # read contents and checksum
            buf = self.waitfor(length+1)
            checksum = buf[-1]
            data = struct.unpack('!%dB' % length, buf[:-1])
            # check message integrity
            if 0xFF & sum(data, 0xFF+mid+length+checksum):
                if self.verbose:
                    sys.stderr.write("invalid checksum; discarding data and "
                                     "waiting for next message.\n")
                continue
            if self.verbose:
                print("MT: Got message id 0x%02X (%s) with %d data bytes: "
                      "[%s]" % (mid, getMIDName(mid), length,
                                ' '.join("%02X" % v for v in data)))
            if mid == MID.Error:
                raise MTErrorMessage(data[0])
            return (mid, buf[:-1])
        else:
            raise MTException("could not find message.")

    def write_ack(self, mid, data=b'', n_resend=30, n_read=25):
        """Send a message and read confirmation."""
        for _ in range(n_resend):
            self.write_msg(mid, data)
            for _ in range(n_read):
                mid_ack, data_ack = self.read_msg()
                if mid_ack == (mid+1):
                    break
                elif self.verbose:
                    print("ack (0x%02X) expected, got 0x%02X instead" %
                        (mid+1, mid_ack))
            else:  # inner look not broken
                continue  # retry (send+wait)
            break  # still no luck
        else:
            n_retries = n_resend*n_read
            raise MTException("Ack (0x%02X) expected, MID 0x%02X received "
                              "instead (after %d retries)." % (mid+1, mid_ack,
                                                               n_retries))
        return data_ack

    def _ensure_config_state(self):
        """Switch device to config state if necessary."""
        if self.state != DeviceState.Config:
            self.GoToConfig()

    def _ensure_measurement_state(self):
        """Switch device to measurement state if necessary."""
        if self.state != DeviceState.Measurement:
            self.GoToMeasurement()

    ############################################################
    # High-level functions
    ############################################################
    def Reset(self, go_to_config=False):
        """Reset MT device.

        If go_to_config then send WakeUpAck in order to leave the device in
        config mode.
        """
        self.write_ack(MID.Reset)
        if go_to_config:
            time.sleep(0.01)
            mid, _ = self.read_msg()
            if mid == MID.WakeUp:
                self.write_msg(MID.WakeUpAck)
                self.state = DeviceState.Config
        else:
            self.state = DeviceState.Measurement

    def GoToConfig(self):
        """Place MT device in configuration mode."""
        self.write_ack(MID.GoToConfig)
        self.state = DeviceState.Config

    def GoToMeasurement(self):
        """Place MT device in measurement mode."""
        self._ensure_config_state()
        self.write_ack(MID.GoToMeasurement)
        self.state = DeviceState.Measurement

    def GetDeviceID(self):
        """Get the device identifier."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqDID)
        deviceID, = struct.unpack('!I', data)
        return deviceID

    def GetProductCode(self):
        """Get the product code."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqProductCode)
        return str(data).strip()

    def GetHardwareVersion(self):
        """Get the hardware version."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqHardwareVersion)
        major, minor = struct.unpack('!BB', data)
        return (major, minor)

    def GetFirmwareRev(self):
        """Get the firmware version."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqFWRev)
        if len(data) == 3:
            major, minor, revision = struct.unpack('!BBB', data)
            return (major, minor, revision)
        else:
            # TODO check buildnr/svnrev not sure unsigned
            major, minor, rev, buildnr, svnrev = struct.unpack('!BBBII', data)
            return (major, minor, rev, buildnr, svnrev)

    def RunSelfTest(self):
        """Run the built-in self test."""
        self._ensure_config_state()
        data = self.write_ack(MID.RunSelfTest)
        bit_names = ['accX', 'accY', 'accZ', 'gyrX', 'gyrY', 'gyrZ',
                     'magX', 'magY', 'magZ']
        self_test_results = []
        for i, name in enumerate(bit_names):
            self_test_results.append((name, (data >> i) & 1))
        return self_test_results

    def GetBaudrate(self):
        """Get the current baudrate id of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetBaudrate)
        return data[0]

    def SetBaudrate(self, brid):
        """Set the baudrate of the device using the baudrate id."""
        self._ensure_config_state()
        self.write_ack(MID.SetBaudrate, (brid,))

    def GetErrorMode(self):
        """Get the current error mode of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetErrorMode)
        error_mode, = struct.unpack('!H', data)
        return error_mode

    def SetErrorMode(self, error_mode):
        """Set the error mode of the device.

        The error mode defines the way the device deals with errors (expect
        message errors):
            0x0000: ignore any errors except message handling errors,
            0x0001: in case of missing sampling instance: increase sample
                counter and do not send error message,
            0x0002: in case of missing sampling instance: increase sample
                counter and send error message,
            0x0003: in case of non-message handling error, an error message is
                sent and the device will go into Config State.
        """
        self._ensure_config_state()
        data = struct.pack('!H', error_mode)
        self.write_ack(MID.SetErrorMode, data)

    def GetOptionFlags(self):
        """Get the option flags (MTi-1 series)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOptionFlags)
        flags, = struct.unpack('!I', data)
        return flags

    def SetOptionFlags(self, set_flags, clear_flags):
        """Set the option flags (MTi-1 series)."""
        self._ensure_config_state()
        data = struct.pack('!II', set_flags, clear_flags)
        self.write_ack(MID.SetOptionFlags, data)

    def GetLocationID(self):
        """Get the location ID of the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetLocationID)
        location_id, = struct.unpack('!H', data)
        return location_id

    def SetLocationID(self, location_id):
        """Set the location ID of the device (arbitrary)."""
        self._ensure_config_state()
        data = struct.pack('!H', location_id)
        self.write_ack(MID.SetLocationID, data)

    def RestoreFactoryDefaults(self):
        """Restore MT device configuration to factory defaults (soft version).
        """
        self._ensure_config_state()
        self.write_ack(MID.RestoreFactoryDef)

    def GetTransmitDelay(self):
        """Get the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetTransmitDelay)
        transmit_delay, = struct.unpack('!H', data)
        return transmit_delay

    def SetTransmitDelay(self, transmit_delay):
        """Set the transmission delay (only RS485)."""
        self._ensure_config_state()
        data = struct.pack('!H', transmit_delay)
        self.write_ack(MID.SetTransmitDelay, data)

    def GetSyncSettings(self):
        """Get the synchronisation settings."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetSyncSettings)
        sync_settings = [struct.unpack('!BBBBHHHH', data[o:o+12])
                         for o in range(0, len(data), 12)]
        return sync_settings

    def SetSyncSettings(self, sync_settings):
        """Set the synchronisation settings (mark IV)"""
        self._ensure_config_state()
        data = b''.join(struct.pack('!BBBBHHHH', *sync_setting)
                        for sync_setting in sync_settings)
        self.write_ack(MID.SetSyncSettings, data)

    def GetConfiguration(self):
        """Ask for the current configuration of the MT device."""
        self._ensure_config_state()
        config = self.write_ack(MID.ReqConfiguration)
        try:
            masterID, period, skipfactor, _, _, _, date, time, num, deviceID,\
                length, mode, settings =\
                struct.unpack('!IHHHHI8s8s32x32xHIHHI8x', config)
        except struct.error:
            raise MTException("could not parse configuration.")
        self.mode = mode
        self.settings = settings
        self.length = length
        if self.length <= 254:
            self.header = b'\xFA\xFF\x32' + struct.pack('!B', self.length)
        else:
            self.header = b'\xFA\xFF\x32\xFF' + struct.pack('!H', self.length)
        conf = {'output-mode': mode,
                'output-settings': settings,
                'length': length,
                'period': period,
                'skipfactor': skipfactor,
                'Master device ID': masterID,
                'date': date,
                'time': time,
                'number of devices': num,
                'device ID': deviceID}
        return conf

    def GetOutputConfiguration(self):
        """Get the output configuration of the device (mark IV)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputConfiguration)
        output_configuration = [struct.unpack('!HH', data[o:o+4])
                                for o in range(0, len(data), 4)]
        return output_configuration

    def SetOutputConfiguration(self, output_configuration):
        """Set the output configuration of the device (mark IV)."""
        self._ensure_config_state()
        data = b''.join(struct.pack('!HH', *output)
                        for output in output_configuration)
        self.write_ack(MID.SetOutputConfiguration, data)

    def GetStringOutputType(self):
        """Get the NMEA data output configuration."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetStringOutputType)
        string_output_type, = struct.unpack('!H', data)
        return string_output_type

    def SetStringOutputType(self, string_output_type):
        """Set the configuration of the NMEA data output."""
        self._ensure_config_state()
        data = struct.pack('!H', string_output_type)
        self.write_ack(MID.SetStringOutputType, data)

    def GetPeriod(self):
        """Get the sampling period."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetPeriod)
        period, = struct.unpack('!H', data)
        return period

    def SetPeriod(self, period):
        """Set the sampling period."""
        self._ensure_config_state()
        data = struct.pack('!H', period)
        self.write_ack(MID.SetPeriod, data)

    def GetAlignmentRotation(self, parameter):
        """Get the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!B', parameter)
        data = self.write_ack(MID.SetAlignmentRotation, data)
        if len(data) == 16:  # fix for older firmwares
            q0, q1, q2, q3 = struct.unpack('!ffff', data)
            return parameter, (q0, q1, q2, q3)
        elif len(data) == 17:
            param, q0, q1, q2, q3 = struct.unpack('!Bffff', data)
            return param, (q0, q1, q2, q3)
        else:
            raise MTException('Could not parse ReqAlignmentRotationAck message:'
                              ' wrong size of message (%d instead of either 16 '
                              'or 17).' % len(data))

    def SetAlignmentRotation(self, parameter, quaternion):
        """Set the object alignment.

        parameter indicates the desired alignment quaternion:
            0 for sensor alignment (RotSensor),
            1 for local alignment (RotLocal).
        """
        self._ensure_config_state()
        data = struct.pack('!Bffff', parameter, *quaternion)
        self.write_ack(MID.SetAlignmentRotation, data)

    def GetOutputMode(self):
        """Get current output mode."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputMode)
        self.mode, = struct.unpack('!H', data)
        return self.mode

    def SetOutputMode(self, mode):
        """Select which information to output."""
        self._ensure_config_state()
        data = struct.pack('!H', mode)
        self.write_ack(MID.SetOutputMode, data)
        self.mode = mode

    def GetExtOutputMode(self):
        """Get current extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetExtOutputMode)
        ext_mode, = struct.unpack('!H', data)
        return ext_mode

    def SetExtOutputMode(self, ext_mode):
        """Set extended output mode (for alternative UART)."""
        self._ensure_config_state()
        data = struct.pack('!H', ext_mode)
        self.write_ack(MID.SetExtOutputMode, data)

    def GetOutputSettings(self):
        """Get current output mode."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetOutputSettings)
        self.settings, = struct.unpack('!I', data)
        return self.settings

    def SetOutputSettings(self, settings):
        """Select how to output the information."""
        self._ensure_config_state()
        data = struct.pack('!I', settings)
        self.write_ack(MID.SetOutputSettings, data)
        self.settings = settings

    def SetOutputSkipFactor(self, skipfactor):  # deprecated
        """Set the output skip factor."""
        self._ensure_config_state()
        data = struct.pack('!H', skipfactor)
        self.write_ack(DeprecatedMID.SetOutputSkipFactor, data)

    def ReqDataLength(self):  # deprecated
        """Get data length for mark III devices."""
        self._ensure_config_state()
        try:
            data = self.write_ack(DeprecatedMID.ReqDataLength)
        except MTErrorMessage as e:
            if e.code == 0x04:
                sys.stderr.write("ReqDataLength message is deprecated and not "
                                 "recognised by your device.")
                return
            raise e
        self.length, = struct.unpack('!H', data)
        if self.length <= 254:
            self.header = b'\xFA\xFF\x32' + struct.pack('!B', self.length)
        else:
            self.header = b'\xFA\xFF\x32\xFF' + struct.pack('!H', self.length)
        return self.length

    def GetLatLonAlt(self):
        """Get the stored position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = self.write_ack(MID.SetLatLonAlt)
        if len(data) == 24:
            lat, lon, alt = struct.unpack('!ddd', data)
        elif len(data) == 12:
            lat, lon, alt = struct.unpack('!fff', data)
        else:
            raise MTException('Could not parse ReqLatLonAltAck message: wrong'
                              'size of message.')
        return (lat, lon, alt)

    def SetLatLonAlt(self, lat, lon, alt):
        """Set the position of the device.
        It is used internally for local magnetic declination and local gravity.
        """
        self._ensure_config_state()
        data = struct.pack('!ddd', lat, lon, alt)
        self.write_ack(MID.SetLatLonAlt, data)

    def GetAvailableScenarios(self):
        """Get the available XKF scenarios on the device."""
        self._ensure_config_state()
        data = self.write_ack(MID.ReqAvailableScenarios)
        scenarios = []
        try:
            for i in range(len(data)//22):
                scenario_type, version, label =\
                    struct.unpack('!BB20s', data[22*i:22*(i+1)])
                scenarios.append((scenario_type, version, label.strip()))
            # available XKF scenarios
            self.scenarios = scenarios
        except struct.error:
            raise MTException("could not parse the available XKF scenarios.")
        return scenarios

    def GetCurrentScenario(self):
        """Get the ID of the currently used XKF scenario."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetCurrentScenario)
        _, self.scenario_id = struct.unpack('!BB', data)  # version, id
        return self.scenario_id

    def SetCurrentScenario(self, scenario_id):
        """Set the XKF scenario to use."""
        self._ensure_config_state()
        data = struct.pack('!BB', 0, scenario_id)  # version, id
        self.write_ack(MID.SetCurrentScenario, data)

    # New names in mk5
    GetAvailableFilterProfiles = GetAvailableScenarios
    GetFilterProfile = GetCurrentScenario
    SetFilterProfile = SetCurrentScenario

    def GetGnssPlatform(self):
        """Get the current GNSS navigation filter settings."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetGnssPlatform)
        platform, = struct.unpack('!H', data)
        return platform

    def SetGnssPlatform(self, platform):
        """Set the GNSS navigation filter settings."""
        self._ensure_config_state()
        data = struct.pack('!H', platform)
        self.write_ack(MID.SetGnssPlatform, data)

    def ResetOrientation(self, code):
        """Reset the orientation.

        Code can take several values:
            0x0000: store current settings (only in config mode),
            0x0001: heading reset (NOT supported by MTi-G),
            0x0003: object reset.
        """
        data = struct.pack('!H', code)
        self.write_ack(MID.ResetOrientation, data)

    def SetNoRotation(self, duration):
        """Initiate the "no rotation" procedure to estimate gyro biases."""
        self._ensure_measurement_state()
        data = struct.pack('!H', duration)
        self.write_ack(MID.SetNoRotation, data)

    def GetUTCTime(self):
        """Get UTC time from device."""
        self._ensure_config_state()
        data = self.write_ack(MID.SetUTCTime)
        ns, year, month, day, hour, minute, second, flag = \
            struct.unpack('!IHBBBBBB', data)
        return (ns, year, month, day, hour, minute, second, flag)

    def SetUTCTime(self, ns, year, month, day, hour, minute, second, flag):
        """Set UTC time on the device."""
        self._ensure_config_state()
        data = struct.pack('!IHBBBBBB', ns, year, month, day, hour, minute,
                           second, flag)  # no clue what setting flag can mean
        self.write_ack(MID.SetUTCTime, data)

    def AdjustUTCTime(self, ticks):
        """Adjust UTC Time of device using correction ticks (0.1 ms)."""
        self._ensure_config_state()
        data = struct.pack('!i', ticks)
        self.write(MID.AdjustUTCTime, data)  # no ack mentioned in the doc

    def IccCommand(self, command):
        """Command of In-run Compass Calibration (ICC)."""
        if command not in (0, 1, 2, 3):
            raise MTException("unknown ICC command 0x%02X" % command)
        cmd_data = struct.pack('!B', command)
        res_data = self.write_ack(MID.IccCommand, cmd_data)
        cmd_ack = struct.unpack('!B', res_data[:1])
        payload = res_data[1:]
        if cmd_ack != command:
            raise MTException("expected ack of command 0x%02X; got 0x%02X "
                              "instead" % (command, cmd_ack))
        if cmd_ack == 0:
            return
        elif cmd_ack == 1:
            ddt_value, dimension, status = struct.unpack('!fBB', payload)
            return ddt_value, dimension, status
        elif cmd_ack == 2:
            return
        elif cmd_ack == 3:
            state = struct.unpack('!B', payload)
            return state

    ############################################################
    # High-level utility functions
    ############################################################
    def configure_legacy(self, mode, settings, period=None, skipfactor=None):
        """Configure the mode and settings of the MT device in legacy mode."""
        try:
            # switch mark IV devices to legacy mode
            self.SetOutputConfiguration([(0x0000, 0)])
        except MTErrorMessage as e:
            if e.code == 0x04:
                # mark III device
                pass
            else:
                raise
        except MTException as e:
            if self.verbose:
                print("no ack received while switching from MTData2 to MTData.")
            pass  # no ack???
        self.SetOutputMode(mode)
        self.SetOutputSettings(settings)
        if period is not None:
            self.SetPeriod(period)
        if skipfactor is not None:
            self.SetOutputSkipFactor(skipfactor)
        self.GetConfiguration()

    def auto_config_legacy(self):
        """Read configuration from device in legacy mode."""
        self.GetConfiguration()
        return self.mode, self.settings, self.length

    def read_measurement(self, mode=None, settings=None):
        self._ensure_measurement_state()
        # getting data
        # data = self.read_data_msg()
        mid, data = self.read_msg()
        if mid == MID.MTData:
            return self.parse_MTData(data, mode, settings)
        elif mid == MID.MTData2:
            return self.parse_MTData2(data)
        else:
            raise MTException("unknown data message: mid=0x%02X (%s)." %
                              (mid, getMIDName(mid)))

    def parse_MTData2(self, data):
        # Functions to parse each type of packet
        def parse_temperature(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Temperature
                o['Temp'], = struct.unpack('!'+ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_timestamp(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # UTC Time
                o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'],\
                    o['Minute'], o['Second'], o['Flags'] =\
                    struct.unpack('!LHBBBBBB', content)
            elif (data_id & 0x00F0) == 0x20:  # Packet Counter
                o['PacketCounter'], = struct.unpack('!H', content)
            elif (data_id & 0x00F0) == 0x30:  # Integer Time of Week
                o['TimeOfWeek'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x40:  # GPS Age  # deprecated
                o['gpsAge'], = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x50:  # Pressure Age  # deprecated
                o['pressureAge'], = struct.unpack('!B', content)
            elif (data_id & 0x00F0) == 0x60:  # Sample Time Fine
                o['SampleTimeFine'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x70:  # Sample Time Coarse
                o['SampleTimeCoarse'], = struct.unpack('!L', content)
            elif (data_id & 0x00F0) == 0x80:  # Frame Range
                o['startFrame'], o['endFrame'] = struct.unpack('!HH', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_orientation_data(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Quaternion
                o['Q0'], o['Q1'], o['Q2'], o['Q3'] = struct.unpack('!'+4*ffmt,
                                                                   content)
            elif (data_id & 0x00F0) == 0x20:  # Rotation Matrix
                o['a'], o['b'], o['c'], o['d'], o['e'], o['f'], o['g'],\
                    o['h'], o['i'] = struct.unpack('!'+9*ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Euler Angles
                o['Roll'], o['Pitch'], o['Yaw'] = struct.unpack('!'+3*ffmt,
                                                                content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_pressure(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Baro pressure
                o['Pressure'], = struct.unpack('!L', content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_acceleration(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Delta V
                o['Delta v.x'], o['Delta v.y'], o['Delta v.z'] = \
                    struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Acceleration
                o['accX'], o['accY'], o['accZ'] = \
                    struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Free Acceleration
                o['freeAccX'], o['freeAccY'], o['freeAccZ'] = \
                    struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # AccelerationHR
                o['accX'], o['accY'], o['accZ'] = \
                    struct.unpack('!'+3*ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_position(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Altitude MSL  # deprecated
                o['altMsl'], = struct.unpack('!'+ffmt, content)
            elif (data_id & 0x00F0) == 0x20:  # Altitude Ellipsoid
                o['altEllipsoid'], = struct.unpack('!'+ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Position ECEF
                o['ecefX'], o['ecefY'], o['ecefZ'] = \
                    struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # LatLon
                o['lat'], o['lon'] = struct.unpack('!'+2*ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GNSS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # GNSS PVT data
                o['itow'], o['year'], o['month'], o['day'], o['hour'],\
                    o['min'], o['sec'], o['valid'], o['tAcc'], o['nano'],\
                    o['fixtype'], o['flags'], o['numSV'], o['lon'], o['lat'],\
                    o['height'], o['hMSL'], o['hAcc'], o['vAcc'], o['velN'],\
                    o['velE'], o['velD'], o['gSpeed'], o['headMot'],\
                    o['sAcc'], o['headAcc'], o['headVeh'], o['gdop'],\
                    o['pdop'], o['tdop'], o['vdop'], o['hdop'], o['ndop'],\
                    o['edop'] =\
                    struct.unpack('!IHBBBBBBIiBBBxiiiiIIiiiiiIIiHHHHHHH',
                                  content)
                # scaling correction
                o['lon'] *= 1e-7
                o['lat'] *= 1e-7
                o['headMot'] *= 1e-5
                o['headVeh'] *= 1e-5
                o['gdop'] *= 0.01
                o['pdop'] *= 0.01
                o['tdop'] *= 0.01
                o['vdop'] *= 0.01
                o['hdop'] *= 0.01
                o['ndop'] *= 0.01
                o['edop'] *= 0.01
            elif (data_id & 0x00F0) == 0x20:  # GNSS satellites info
                o['iTOW'], o['numSvs'] = struct.unpack('!LBxxx', content[:8])
                svs = []
                ch = {}
                for i in range(o['numSvs']):
                    ch['gnssId'], ch['svId'], ch['cno'], ch['flags'] = \
                        struct.unpack('!BBBB', content[8+4*i:12+4*i])
                    svs.append(ch)
                o['svs'] = svs
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_angular_velocity(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x20:  # Rate of Turn
                o['gyrX'], o['gyrY'], o['gyrZ'] = \
                    struct.unpack('!'+3*ffmt, content)
            elif (data_id & 0x00F0) == 0x30:  # Delta Q
                o['Delta q0'], o['Delta q1'], o['Delta q2'], o['Delta q3'] = \
                    struct.unpack('!'+4*ffmt, content)
            elif (data_id & 0x00F0) == 0x40:  # RateOfTurnHR
                o['gyrX'], o['gyrY'], o['gyrZ'] = \
                    struct.unpack('!'+3*ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_GPS(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x30:  # DOP
                o['iTOW'], g, p, t, v, h, n, e = \
                    struct.unpack('!LHHHHHHH', content)
                o['gDOP'], o['pDOP'], o['tDOP'], o['vDOP'], o['hDOP'], \
                    o['nDOP'], o['eDOP'] = 0.01*g, 0.01*p, 0.01*t, \
                    0.01*v, 0.01*h, 0.01*n, 0.01*e
            elif (data_id & 0x00F0) == 0x40:  # SOL
                o['iTOW'], o['fTOW'], o['Week'], o['gpsFix'], o['Flags'], \
                    o['ecefX'], o['ecefY'], o['ecefZ'], o['pAcc'], \
                    o['ecefVX'], o['ecefVY'], o['ecefVZ'], o['sAcc'], \
                    o['pDOP'], o['numSV'] = \
                    struct.unpack('!LlhBBlllLlllLHxBx', content)
                # scaling correction
                o['pDOP'] *= 0.01
            elif (data_id & 0x00F0) == 0x80:  # Time UTC
                o['iTOW'], o['tAcc'], o['nano'], o['year'], o['month'], \
                    o['day'], o['hour'], o['min'], o['sec'], o['valid'] = \
                    struct.unpack('!LLlHBBBBBB', content)
            elif (data_id & 0x00F0) == 0xA0:  # SV Info
                o['iTOW'], o['numCh'] = struct.unpack('!LBxxx', content[:8])
                channels = []
                ch = {}
                for i in range(o['numCh']):
                    ch['chn'], ch['svid'], ch['flags'], ch['quality'], \
                        ch['cno'], ch['elev'], ch['azim'], ch['prRes'] = \
                        struct.unpack('!BBBBBbhl', content[8+12*i:20+12*i])
                    channels.append(ch)
                o['channels'] = channels
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_SCR(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # ACC+GYR+MAG+Temperature
                o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], \
                    o['gyrZ'], o['magX'], o['magY'], o['magZ'], o['Temp'] = \
                    struct.unpack("!9Hh", content)
            elif (data_id & 0x00F0) == 0x20:  # Gyro Temperature
                o['tempGyrX'], o['tempGyrY'], o['tempGyrZ'] = \
                    struct.unpack("!hhh", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_analog_in(data_id, content, ffmt):  # deprecated
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Analog In 1
                o['analogIn1'], = struct.unpack("!H", content)
            elif (data_id & 0x00F0) == 0x20:  # Analog In 2
                o['analogIn2'], = struct.unpack("!H", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_magnetic(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x20:  # Magnetic Field
                o['magX'], o['magY'], o['magZ'] = \
                    struct.unpack("!3"+ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_velocity(data_id, content, ffmt):
            o = {}
            if (data_id & 0x000C) == 0x00:  # ENU
                o['frame'] = 'ENU'
            elif (data_id & 0x000C) == 0x04:  # NED
                o['frame'] = 'NED'
            elif (data_id & 0x000C) == 0x08:  # NWU
                o['frame'] = 'NWU'
            if (data_id & 0x00F0) == 0x10:  # Velocity XYZ
                o['velX'], o['velY'], o['velZ'] = \
                    struct.unpack("!3"+ffmt, content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        def parse_status(data_id, content, ffmt):
            o = {}
            if (data_id & 0x00F0) == 0x10:  # Status Byte
                o['StatusByte'], = struct.unpack("!B", content)
            elif (data_id & 0x00F0) == 0x20:  # Status Word
                o['StatusWord'], = struct.unpack("!L", content)
            elif (data_id & 0x00F0) == 0x40:  # RSSI  # deprecated
                o['RSSI'], = struct.unpack("!b", content)
            else:
                raise MTException("unknown packet: 0x%04X." % data_id)
            return o

        # data object
        output = {}
        while len(data) > 0:
            try:
                data_id, size = struct.unpack('!HB', data[:3])
                if (data_id & 0x0003) == 0x3:
                    float_format = 'd'
                elif (data_id & 0x0003) == 0x0:
                    float_format = 'f'
                else:
                    raise MTException("fixed point precision not supported.")
                content = data[3:3+size]
                data = data[3+size:]
                group = data_id & 0xF800
                ffmt = float_format
                if group == XDIGroup.Temperature:
                    output.setdefault('Temperature', {}).update(
                        parse_temperature(data_id, content, ffmt))
                elif group == XDIGroup.Timestamp:
                    output.setdefault('Timestamp', {}).update(
                        parse_timestamp(data_id, content, ffmt))
                elif group == XDIGroup.OrientationData:
                    output.setdefault('Orientation Data', {}).update(
                        parse_orientation_data(data_id, content, ffmt))
                elif group == XDIGroup.Pressure:
                    output.setdefault('Pressure', {}).update(
                        parse_pressure(data_id, content, ffmt))
                elif group == XDIGroup.Acceleration:
                    output.setdefault('Acceleration', {}).update(
                        parse_acceleration(data_id, content, ffmt))
                elif group == XDIGroup.Position:
                    output.setdefault('Position', {}).update(
                        parse_position(data_id, content, ffmt))
                elif group == XDIGroup.GNSS:
                    output.setdefault('GNSS', {}).update(
                        parse_GNSS(data_id, content, ffmt))
                elif group == XDIGroup.AngularVelocity:
                    output.setdefault('Angular Velocity', {}).update(
                        parse_angular_velocity(data_id, content, ffmt))
                elif group == XDIGroup.GPS:
                    output.setdefault('GPS', {}).update(
                        parse_GPS(data_id, content, ffmt))
                elif group == XDIGroup.SensorComponentReadout:
                    output.setdefault('SCR', {}).update(
                        parse_SCR(data_id, content, ffmt))
                elif group == XDIGroup.AnalogIn:  # deprecated
                    output.setdefault('Analog In', {}).update(
                        parse_analog_in(data_id, content, ffmt))
                elif group == XDIGroup.Magnetic:
                    output.setdefault('Magnetic', {}).update(
                        parse_magnetic(data_id, content, ffmt))
                elif group == XDIGroup.Velocity:
                    output.setdefault('Velocity', {}).update(
                        parse_velocity(data_id, content, ffmt))
                elif group == XDIGroup.Status:
                    output.setdefault('Status', {}).update(
                        parse_status(data_id, content, ffmt))
                else:
                    raise MTException("unknown XDI group: 0x%04X." % group)
            except struct.error:
                raise MTException("couldn't parse MTData2 message (data_id: "
				  "0x%04X, size: %d)." % (data_id, size))
        return output

    def parse_MTData(self, data, mode=None, settings=None):
        """Read and parse a legacy measurement packet."""
        # getting mode
        if mode is None:
            mode = self.mode
        if settings is None:
            settings = self.settings
        # data object
        output = {}
        try:
            # raw IMU first
            if mode & OutputMode.RAW:
                o = {}
                o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], \
                    o['gyrZ'], o['magX'], o['magY'], o['magZ'], o['temp'] =\
                    struct.unpack('!10H', data[:20])
                data = data[20:]
                output['RAW'] = o
            # raw GPS second
            if mode & OutputMode.RAWGPS:
                o = {}
                o['Press'], o['bPrs'], o['ITOW'], o['LAT'], o['LON'],\
                    o['ALT'], o['VEL_N'], o['VEL_E'], o['VEL_D'], o['Hacc'],\
                    o['Vacc'], o['Sacc'], o['bGPS'] =\
                    struct.unpack('!HBI6i3IB', data[:44])
                data = data[44:]
                output['RAWGPS'] = o
            # temperature
            if mode & OutputMode.Temp:
                temp, = struct.unpack('!f', data[:4])
                data = data[4:]
                output['Temp'] = temp
            # calibrated data
            if mode & OutputMode.Calib:
                o = {}
                if (settings & OutputSettings.Coordinates_NED):
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                if not (settings & OutputSettings.CalibMode_GyrMag):
                    o['accX'], o['accY'], o['accZ'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                if not (settings & OutputSettings.CalibMode_AccMag):
                    o['gyrX'], o['gyrY'], o['gyrZ'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                if not (settings & OutputSettings.CalibMode_AccGyr):
                    o['magX'], o['magY'], o['magZ'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                output['Calib'] = o
            # orientation
            if mode & OutputMode.Orient:
                o = {}
                if (settings & OutputSettings.Coordinates_NED):
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                if settings & OutputSettings.OrientMode_Euler:
                    o['roll'], o['pitch'], o['yaw'] = struct.unpack('!3f',
                                                                    data[:12])
                    data = data[12:]
                elif settings & OutputSettings.OrientMode_Matrix:
                    a, b, c, d, e, f, g, h, i = struct.unpack('!9f',
                                                              data[:36])
                    data = data[36:]
                    o['matrix'] = ((a, b, c), (d, e, f), (g, h, i))
                else:  # OutputSettings.OrientMode_Quaternion:
                    q0, q1, q2, q3 = struct.unpack('!4f', data[:16])
                    data = data[16:]
                    o['quaternion'] = (q0, q1, q2, q3)
                output['Orient'] = o
            # auxiliary
            if mode & OutputMode.Auxiliary:
                o = {}
                if not (settings & OutputSettings.AuxiliaryMode_NoAIN1):
                    o['Ain_1'], = struct.unpack('!H', data[:2])
                    data = data[2:]
                if not (settings & OutputSettings.AuxiliaryMode_NoAIN2):
                    o['Ain_2'], = struct.unpack('!H', data[:2])
                    data = data[2:]
                output['Auxiliary'] = o
            # position
            if mode & OutputMode.Position:
                o = {}
                o['Lat'], o['Lon'], o['Alt'] = struct.unpack('!3f', data[:12])
                data = data[12:]
                output['Pos'] = o
            # velocity
            if mode & OutputMode.Velocity:
                o = {}
                if (settings & OutputSettings.Coordinates_NED):
                    o['frame'] = 'NED'
                else:
                    o['frame'] = 'ENU'
                o['Vel_X'], o['Vel_Y'], o['Vel_Z'] = struct.unpack('!3f',
                                                                   data[:12])
                data = data[12:]
                output['Vel'] = o
            # status
            if mode & OutputMode.Status:
                status, = struct.unpack('!B', data[:1])
                data = data[1:]
                output['Stat'] = status
            # sample counter
            if settings & OutputSettings.Timestamp_SampleCnt:
                TS, = struct.unpack('!H', data[:2])
                data = data[2:]
                output['Sample'] = TS
            # UTC time
            if settings & OutputSettings.Timestamp_UTCTime:
                o = {}
                o['ns'], o['Year'], o['Month'], o['Day'], o['Hour'],\
                    o['Minute'], o['Second'], o['Flags'] = struct.unpack(
                        '!ihbbbbb', data[:12])
                data = data[12:]
                output['Timestamp'] = o
            # TODO at that point data should be empty
        except struct.error as e:
            raise MTException("could not parse MTData message.")
        if len(data) > 0:
            raise MTException("could not parse MTData message (too long).")
        return output

    def ChangeBaudrate(self, baudrate):
        """Change the baudrate, reset the device and reopen communication."""
        brid = Baudrates.get_BRID(baudrate)
        self.SetBaudrate(brid)
        self.Reset()
        # self.device.flush()
        self.device.baudrate = baudrate
        # self.device.flush()
        time.sleep(0.01)
        self.read_msg()
        self.write_msg(MID.WakeUpAck)


################################################################
# Auto detect port
################################################################
def find_devices(timeout=0.002, verbose=False, initial_wait=0.1):
    mtdev_list = []
    for port in glob.glob("/dev/tty*S*"):
        if verbose:
            print("Trying '%s'" % port)
        try:
            br = find_baudrate(port, timeout, verbose, initial_wait)
            if br:
                mtdev_list.append((port, br))
        except MTException:
            pass
    return mtdev_list


################################################################
# Auto detect baudrate
################################################################
def find_baudrate(port, timeout=0.002, verbose=False, initial_wait=0.1):
    baudrates = (115200, 460800, 921600, 230400, 57600, 38400, 19200, 9600)
    for br in baudrates:
        if verbose:
            print("Trying %d bd:" % br, end=' ')
            sys.stdout.flush()
        try:
            mt = MTDevice(port, br, timeout=timeout, verbose=verbose,
                          initial_wait=initial_wait)
        except serial.SerialException:
            if verbose:
                print("fail: unable to open device.")
            raise MTException("unable to open %s" % port)
        try:
            mt.GoToConfig()
            mt.GoToMeasurement()
            if verbose:
                print("ok.")
            return br
        except MTException:
            if verbose:
                print("fail.")
