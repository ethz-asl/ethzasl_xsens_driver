#!/usr/bin/env python
from __future__ import print_function

import datetime
import getopt
import pprint
import re
from serial import SerialException
import sys

from xsens_driver.mtdef import MTException, MTErrorMessage, MTTimeoutException, OutputMode, OutputSettings
from xsens_driver.mtdevice import find_devices, find_baudrate, MTDevice

################################################################
# Documentation for stand alone usage
################################################################
def usage():
    print("""MT device driver.
Usage:
    ./mtdevice.py [commands] [opts]

Commands:
    -h, --help
        Print this help and quit.
    -r, --reset
        Reset device to factory defaults.
    -a, --change-baudrate=NEW_BAUD
        Change baudrate from BAUD (see below) to NEW_BAUD.
    -c, --configure=OUTPUT
        Configure the device (see OUTPUT description below).
    -e, --echo
        Print MTData. It is the default if no other command is supplied.
    -i, --inspect
        Print current MT device configuration.
    -x, --xkf-scenario=ID
        Change the current XKF scenario.
    -l, --legacy-configure
        Configure the device in legacy mode (needs MODE and SETTINGS arguments
        below).
    -v, --verbose
        Verbose output.
    -y, --synchronization=settings (see below)
        Configure the synchronization settings of each sync line (see below).
    -u, --utc-time=time (see below)
        Set the UTC time buffer of the device.
    -g, --gnss-platform=platform
        Change the GNSS navigation filter settings (check the documentation).
    -o, --option-flags=flags (see below)
        Set the option flags.
    -j, --icc-command=command (see below)
        Send command to the In-run Compass Calibration.

Generic options:
    -d, --device=DEV
        Serial interface of the device (default: /dev/ttyUSB0). If 'auto', then
        all serial ports are tested at all baudrates and the first
        suitable device is used.
    -b, --baudrate=BAUD
        Baudrate of serial interface (default: 115200). If 0, then all
        rates are tried until a suitable one is found.
    -t, --timeout=TIMEOUT
        Timeout of serial communication in second (default: 0.002).
    -w, --initial-wait=WAIT
        Initial wait to allow device to be ready in second (default: 0.1).

Configuration option:
    OUTPUT
        The format is a sequence of "<group><type><frequency>?<format>?"
        separated by commas.
        The frequency and format are optional.
        The groups and types can be:
            t  temperature (max frequency: 1 Hz):
                tt  temperature
            i  timestamp (max frequency: 2000 Hz):
                iu  UTC time
                ip  packet counter
                ii  Integer Time of the Week (ITOW)
                if  sample time fine
                ic  sample time coarse
                ir  frame range
            o  orientation data (max frequency: 400 Hz):
                oq  quaternion
                om  rotation matrix
                oe  Euler angles
            b  pressure (max frequency: 50 Hz):
                bp  baro pressure
            a  acceleration (max frequency: 2000 Hz (see documentation)):
                ad  delta v
                aa  acceleration
                af  free acceleration
                ah  acceleration HR (max frequency 1000 Hz)
            p  position (max frequency: 400 Hz):
                pa  altitude ellipsoid
                pp  position ECEF
                pl  latitude longitude
            n  GNSS (max frequency: 4 Hz):
                np  GNSS PVT data
                ns  GNSS satellites info
            w  angular velocity (max frequency: 2000 Hz (see documentation)):
                wr  rate of turn
                wd  delta q
                wh  rate of turn HR (max frequency 1000 Hz)
            g  GPS (max frequency: 4 Hz):
                gd  DOP
                gs  SOL
                gu  time UTC
                gi  SV info
            r  Sensor Component Readout (max frequency: 2000 Hz):
                rr  ACC, GYR, MAG, temperature
                rt  Gyro temperatures
            m  Magnetic (max frequency: 100 Hz):
                mf  magnetic Field
            v  Velocity (max frequency: 400 Hz):
                vv  velocity XYZ
            s  Status (max frequency: 2000 Hz):
                sb  status byte
                sw  status word
        Frequency is specified in decimal and is assumed to be the maximum
        frequency if it is omitted.
        Format is a combination of the precision for real valued numbers and
        coordinate system:
            precision:
                f  single precision floating point number (32-bit) (default)
                d  double precision floating point number (64-bit)
            coordinate system:
                e  East-North-Up (default)
                n  North-East-Down
                w  North-West-Up
        Examples:
            The default configuration for the MTi-1/10/100 IMUs can be
            specified either as:
                "wd,ad,mf,ip,if,sw"
            or
                "wd2000fe,ad2000fe,mf100fe,ip2000,if2000,sw2000"
            For getting quaternion orientation in float with sample time:
                "oq400fw,if2000"
            For longitude, latitude, altitude and orientation (on MTi-G-700):
                "pl400fe,pa400fe,oq400fe"

Synchronization settings:
    The format follows the xsens protocol documentation. All fields are
    required and separated by commas.
    Note: The entire synchronization buffer is wiped every time a new one
          is set, so it is necessary to specify the settings of multiple
          lines at once.
    It also possible to clear the synchronization with the argument "clear"

        Function (see manual for details):
             3 Trigger indication
             4 Interval Transition Measurement
             8 SendLatest
             9 ClockBiasEstimation
            11 StartSampling
        Line (manual for details):
            0 ClockIn
            1 GPSClockIn (only available for 700/710)
            2 Input Line (SyncIn)
            4 SyncOut
            5 ExtTimepulseIn (only available for 700/710)
            6 Software (only available for SendLatest with ReqData message)
        Polarity:
            1 Positive pulse/ Rising edge
            2 Negative pulse/ Falling edge
            3 Both/ Toggle
        Trigger Type:
            0 multiple times
            1 once
        Skip First (unsigned_int):
            Number of initial events to skip before taking actions
        Skip Factor (unsigned_int):
            Number of events to skip before taking action again
            Ignored with ReqData.
        Pulse Width (unsigned_int):
            Ignored for SyncIn.
            For SyncOut, the width of the generated pulse in 100 microseconds
            unit. Ignored for Toggle pulses.
        Delay:
            Delay after receiving a sync pulse to taking action,
            100 microseconds units, range [0...600000]
        Clock Period:
            Reference clock period in milliseconds for ClockBiasEstimation
        Offset:
            Offset from event to pulse generation.
            100 microseconds unit, range [-30000...+30000]

    Examples:
        For changing the sync setting of the SyncIn line to trigger indication
        with rising edge, one time triggering and no skipping and delay. Enter
        the settings as:
            "3,2,1,1,0,0,0,0"

        Note a number is still in the place for pulse width despite it being
        ignored.

        To set multiple lines at once:
        ./mtdevice.py -y 3,2,1,0,0,0,0,0 -y 9,0,1,0,0,0,10,0

        To clear the synchronization settings of MTi
        ./mtdevice.py -y clear

UTC time settings:
    There are two ways to set the UTCtime for the MTi.
    Option #1: set MTi to the current UTC time based on local system time with
               the option 'now'
    Option #2: set MTi to a specified UTC time
        The time fields are set as follows:
            year: range [1999,2099]
            month: range [1,12]
            day: day of the month, range [1,31]
            hour: hour of the day, range [0,23]
            min: minute of the hour, range [0,59]
            sec: second of the minute, range [0,59]
            ns: nanosecond of the second, range [0,1000000000]
            flag:
                1: Valid Time of Week
                2: Valid Week Number
                4: valid UTC
            Note: the flag is ignored for --utc-time as it is set by the device
                  itself when connected to a GPS

    Examples:
        Set UTC time for the device:
        ./mtdevice.py -u now
        ./mtdevice.py -u 1999,1,1,0,0,0,0,0

GNSS platform settings:
    Only for MTi-G-700/710 with firmware>=1.7.
    The following two platform settings are listed in the documentation:
        0:  Portable
        8:  Airbone <4g
    Check the XSens documentation before changing anything.

Option flags:
    Several flags can be set or cleared.
    0x00000001  DisableAutoStore: when set, configuration changes are not saved
                    in non-volatile memory (only MTi-1 series)
    0x00000002  DisableAutoMeasurement: when set, device will stay in Config
                    Mode upon start up (only MTi-1 series)
    0x00000004  EnableBeidou: when set, enable Beidou and disable GLONASS (only
                    MTi-G-710)
    0x00000010  EnableAHS: enable Active Heading Stabilization (overrides
                    magnetic reference)
    0x00000080  EnableInRunCompassCalibration: doc is unclear
    The flags provided must be a pair of ored values: the first for flags to be
    set the second for the flags to be cleared.
    Examples:
        Only set DisableAutoStore and DisableAutoMeasurement flags:
            ./mtdevice.py -o 0x03,0x00
        Disable AHS (clear EnableAHS flag):
            ./mtdevice.py -o 0x00,0x10
        Set DisableAutoStore and clear DisableAutoMeasurement:
            ./mtdevice.py -o 0x02,0x01

In-run Compass Calibration commands:
    The idea of ICC is to record magnetic field data during so-called
    representative motion in order to better calibrate the magnetometer and
    improve the fusion.
    Typical usage would be to issue the start command, then move the device
    for some time then issue the stop command. If parameters are acceptable,
    these can be stored using the store command.
    Commands:
        00: Start representative motion
        01: Stop representative motion; return ddt, dimension, and status.
        02: Store ICC parameters
        03: Get representative motion state; return 1 if active
    Check the documentation for more details.

Legacy options:
    -m, --output-mode=MODE
        Legacy mode of the device to select the information to output.
        This is required for 'legacy-configure' command.
        MODE can be either the mode value in hexadecimal, decimal or
        binary form, or a string composed of the following characters
        (in any order):
            t  temperature, [0x0001]
            c  calibrated data, [0x0002]
            o  orientation data, [0x0004]
            a  auxiliary data, [0x0008]
            p  position data (requires MTi-G), [0x0010]
            v  velocity data (requires MTi-G), [0x0020]
            s  status data, [0x0800]
            g  raw GPS mode (requires MTi-G), [0x1000]
            r  raw (incompatible with others except raw GPS), [0x4000]
        For example, use "--output-mode=so" to have status and
        orientation data.
    -s, --output-settings=SETTINGS
        Legacy settings of the device. This is required for 'legacy-configure'
        command.
        SETTINGS can be either the settings value in hexadecimal,
        decimal or binary form, or a string composed of the following
        characters (in any order):
            t  sample count (excludes 'n')
            n  no sample count (excludes 't')
            u  UTC time
            q  orientation in quaternion (excludes 'e' and 'm')
            e  orientation in Euler angles (excludes 'm' and 'q')
            m  orientation in matrix (excludes 'q' and 'e')
            A  acceleration in calibrated data
            G  rate of turn in calibrated data
            M  magnetic field in calibrated data
            i  only analog input 1 (excludes 'j')
            j  only analog input 2 (excludes 'i')
            N  North-East-Down instead of default: X North Z up
        For example, use "--output-settings=tqMAG" for all calibrated
        data, sample counter and orientation in quaternion.
    -p, --period=PERIOD
        Sampling period in (1/115200) seconds (default: 1152).
        Minimum is 225 (1.95 ms, 512 Hz), maximum is 1152
        (10.0 ms, 100 Hz).
        Note that for legacy devices it is the period at which sampling occurs,
        not the period at which messages are sent (see below).

Deprecated options:
    -f, --deprecated-skip-factor=SKIPFACTOR
        Only for mark III devices.
        Number of samples to skip before sending MTData message
        (default: 0).
        The frequency at which MTData message is send is:
            115200/(PERIOD * (SKIPFACTOR + 1))
        If the value is 0xffff, no data is send unless a ReqData request
        is made.
""")


################################################################
# Main function
################################################################
def main():
    # parse command line
    shopts = 'hra:c:eild:b:m:s:p:f:x:vy:u:g:o:j:t:w:'
    lopts = ['help', 'reset', 'change-baudrate=', 'configure=', 'echo',
             'inspect', 'legacy-configure', 'device=', 'baudrate=',
             'output-mode=', 'output-settings=', 'period=',
             'deprecated-skip-factor=', 'xkf-scenario=', 'verbose',
             'synchronization=', 'utc-time=', 'gnss-platform=',
             'option-flags=', 'icc-command=', 'timeout=', 'initial-wait=']
    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], shopts, lopts)
    except getopt.GetoptError as e:
        print(e)
        usage()
        return 1
    # default values
    device = '/dev/ttyUSB0'
    baudrate = 115200
    timeout = 0.002
    initial_wait = 0.1
    mode = None
    settings = None
    period = None
    skipfactor = None
    new_baudrate = None
    new_xkf = None
    actions = []
    verbose = False
    sync_settings = []  # list of synchronization settings

    # filling in arguments
    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            return
        elif o in ('-r', '--reset'):
            actions.append('reset')
        elif o in ('-a', '--change-baudrate'):
            try:
                new_baudrate = int(a)
            except ValueError:
                print("change-baudrate argument must be integer.")
                return 1
            actions.append('change-baudrate')
        elif o in ('-c', '--configure'):
            output_config = get_output_config(a)
            if output_config is None:
                return 1
            actions.append('configure')
        elif o in ('-e', '--echo'):
            actions.append('echo')
        elif o in ('-i', '--inspect'):
            actions.append('inspect')
        elif o in ('-l', '--legacy-configure'):
            actions.append('legacy-configure')
        elif o in ('-x', '--xkf-scenario'):
            try:
                new_xkf = int(a)
            except ValueError:
                print("xkf-scenario argument must be integer.")
                return 1
            actions.append('xkf-scenario')
        elif o in ('-y', '--synchronization'):
            new_sync_settings = get_synchronization_settings(a)
            if new_sync_settings is None:
                return 1
            sync_settings.append(new_sync_settings)
            actions.append('synchronization')
        elif o in ('-u', '--setUTCtime'):
            UTCtime_settings = get_UTCtime(a)
            if UTCtime_settings is None:
                return 1
            actions.append('setUTCtime')
        elif o in ('-d', '--device'):
            device = a
        elif o in ('-b', '--baudrate'):
            try:
                baudrate = int(a)
            except ValueError:
                print("baudrate argument must be integer.")
                return 1
        elif o in ('-m', '--output-mode'):
            mode = get_mode(a)
            if mode is None:
                return 1
        elif o in ('-s', '--output-settings'):
            settings = get_settings(a)
            if settings is None:
                return 1
        elif o in ('-p', '--period'):
            try:
                period = int(a)
            except ValueError:
                print("period argument must be integer.")
                return 1
        elif o in ('-f', '--deprecated-skip-factor'):
            try:
                skipfactor = int(a)
            except ValueError:
                print("skip-factor argument must be integer.")
                return 1
        elif o in ('-v', '--verbose'):
            verbose = True
        elif o in ('-g', '--gnss-platform'):
            platform = get_gnss_platform(a)
            if platform is None:
                return 1
            actions.append('gnss-platform')
        elif o in ('-o', '--option-flags'):
            flag_tuple = get_option_flags(a)
            if flag_tuple is None:
                return 1
            actions.append('option-flags')
        elif o in ('-j', '--icc-command'):
            icc_command = get_icc_command(a)
            if icc_command is None:
                return 1
            actions.append('icc-command')
        elif o in ('-t', '--timeout'):
            try:
                timeout = float(a)
            except ValueError:
                print("timeout argument must be a floating number.")
                return 1
        elif o in ('-w', '--initial-wait'):
            try:
                initial_wait = float(a)
            except ValueError:
                print("initial-wait argument must be a floating number.")
                return 1

    # if nothing else: echo
    if len(actions) == 0:
        actions.append('echo')
    try:
        if device == 'auto':
            devs = find_devices(timeout=timeout, verbose=verbose,
                                initial_wait=initial_wait)
            if devs:
                print("Detected devices:", "".join('\n\t%s @ %d' % (d, p)
                                                   for d, p in devs))
                print("Using %s @ %d" % devs[0])
                device, baudrate = devs[0]
            else:
                print("No suitable device found.")
                return 1
        # find baudrate
        if not baudrate:
            baudrate = find_baudrate(device, timeout=timeout, verbose=verbose,
                                     initial_wait=initial_wait)
        if not baudrate:
            print("No suitable baudrate found.")
            return 1
        # open device
        try:
            mt = MTDevice(device, baudrate, timeout=timeout, verbose=verbose,
                          initial_wait=initial_wait)
        except SerialException:
            raise MTException("unable to open %s" % device)
        # execute actions
        if 'inspect' in actions:
            inspect(mt, device, baudrate)
        if 'change-baudrate' in actions:
            print("Changing baudrate from %d to %d:" % (baudrate,
                                                        new_baudrate), end=' ')
            sys.stdout.flush()
            mt.ChangeBaudrate(new_baudrate)
            print(" Ok")  # should we test that it was actually ok?
        if 'reset' in actions:
            print("Restoring factory defaults", end=' ')
            sys.stdout.flush()
            mt.RestoreFactoryDefaults()
            print(" Ok")  # should we test that it was actually ok?
        if 'configure' in actions:
            print("Changing output configuration", end=' ')
            sys.stdout.flush()
            mt.SetOutputConfiguration(output_config)
            print(" Ok")  # should we test that it was actually ok?
        if 'synchronization' in actions:
            print("Changing synchronization settings", end=' ')
            sys.stdout.flush()
            mt.SetSyncSettings(sync_settings)
            print(" Ok")  # should we test that it was actually ok?
        if 'setUTCtime' in actions:
            print("Setting UTC time in the device", end=' ')
            sys.stdout.flush()
            mt.SetUTCTime(UTCtime_settings[6],
                          UTCtime_settings[0],
                          UTCtime_settings[1],
                          UTCtime_settings[2],
                          UTCtime_settings[3],
                          UTCtime_settings[4],
                          UTCtime_settings[5],
                          UTCtime_settings[7])
            print(" Ok")  # should we test that it was actually ok?
        if 'gnss-platform' in actions:
            print("Setting GNSS platform", end=' ')
            sys.stdout.flush()
            mt.SetGnssPlatform(platform)
            print(" Ok")  # should we test that it was actually ok?
        if 'option-flags' in actions:
            print("Setting option flags", end=' ')
            sys.stdout.flush()
            mt.SetOptionFlags(*flag_tuple)
            print(" Ok")  # should we test that it was actually ok?
        if 'icc-command' in actions:
            icc_command_names = {
                    0: 'start representative motion',
                    1: 'stop representative motion',
                    2: 'store ICC results',
                    3: 'representative motion state'}
            print("Sending ICC command 0x%02X (%s):" % (
                    icc_command, icc_command_names[icc_command]), end=' ')
            sys.stdout.flush()
            res = mt.IccCommand(icc_command)
            if icc_command == 0x00:
                print(" Ok")  # should we test that it was actually ok?
            elif icc_command == 0x01:
                print(res)
            elif icc_command == 0x02:
                print(" Ok")  # should we test that it was actually ok?
            elif icc_command == 0x03:
                res_string = {0: 'representative motion inactive',
                              1: 'representation motion active'}
                print("0x02X (%s)" % (res, res_string.get(res, 'unknown')))
        if 'legacy-configure' in actions:
            if mode is None:
                print("output-mode is require to configure the device in "
                    "legacy mode.")
                return 1
            if settings is None:
                print("output-settings is required to configure the device "
                      "in legacy mode.")
                return 1
            print("Configuring in legacy mode", end=' ')
            sys.stdout.flush()
            mt.configure_legacy(mode, settings, period, skipfactor)
            print(" Ok")        # should we test it was actually ok?
        if 'xkf-scenario' in actions:
            print("Changing XKF scenario", end=' ')
            sys.stdout.flush()
            mt.SetCurrentScenario(new_xkf)
            print("Ok")
        if 'echo' in actions:
            # if (mode is None) or (settings is None):
            #     mode, settings, length = mt.auto_config()
            #     print mode, settings, length
            try:
                while True:
                    print(mt.read_measurement(mode, settings))
            except KeyboardInterrupt:
                pass
    except MTErrorMessage as e:
        print("MTErrorMessage:", e)
    except MTException as e:
        print("MTException:", e)


def inspect(mt, device, baudrate):
    """Inspection."""
    def config_fmt(config):
        """Hexadecimal configuration."""
        return '[%s]' % ', '.join('(0x%04X, %d)' % (mode, freq)
                                  for (mode, freq) in config)

    def hex_fmt(size=4):
        """Factory for hexadecimal representation formatter."""
        fmt = '0x%%0%dX' % (2*size)

        def f(value):
            """Hexadecimal representation."""
            # length of string is twice the size of the value (in bytes)
            return fmt % value
        return f

    def sync_fmt(settings):
        """Synchronization settings: N*12 bytes"""
        return '[%s]' % ', '.join('(0x%02X, 0x%02X, 0x%02X, 0x%02X,'
                                  ' 0x%04X, 0x%04X, 0x%04X, 0x%04X)' % s
                                  for s in settings)

    def try_message(m, f, formater=None, *args, **kwargs):
        print('  %s ' % m, end=' ')
        try:
            if formater is not None:
                print(formater(f(*args, **kwargs)))
            else:
                pprint.pprint(f(*args, **kwargs), indent=4)
        except MTTimeoutException as e:
            print('timeout: might be unsupported by your device.')
        except MTErrorMessage as e:
            if e.code == 0x04:
                print('message unsupported by your device.')
            else:
                raise e
    print("Device: %s at %d Bd:" % (device, baudrate))
    try_message("device ID:", mt.GetDeviceID, hex_fmt(4))
    try_message("product code:", mt.GetProductCode)
    try_message("hardware version:", mt.GetHardwareVersion)
    try_message("firmware revision:", mt.GetFirmwareRev)
    try_message("baudrate:", mt.GetBaudrate)
    try_message("error mode:", mt.GetErrorMode, hex_fmt(2))
    try_message("option flags:", mt.GetOptionFlags, hex_fmt(4))
    try_message("location ID:", mt.GetLocationID, hex_fmt(2))
    try_message("transmit delay:", mt.GetTransmitDelay)
    try_message("synchronization settings:", mt.GetSyncSettings, sync_fmt)
    try_message("general configuration:", mt.GetConfiguration)
    try_message("output configuration (mark IV devices):",
                mt.GetOutputConfiguration, config_fmt)
    try_message("string output type:", mt.GetStringOutputType)
    try_message("period:", mt.GetPeriod)
    try_message("alignment rotation sensor:", mt.GetAlignmentRotation,
                parameter=0)
    try_message("alignment rotation local:", mt.GetAlignmentRotation,
                parameter=1)
    try_message("output mode:", mt.GetOutputMode, hex_fmt(2))
    try_message("extended output mode:", mt.GetExtOutputMode, hex_fmt(2))
    try_message("output settings:", mt.GetOutputSettings, hex_fmt(4))
    try_message("GPS coordinates (lat, lon, alt):", mt.GetLatLonAlt)
    try_message("GNSS platform:", mt.GetGnssPlatform)
    try_message("available scenarios:", mt.GetAvailableScenarios)
    try_message("current scenario ID:", mt.GetCurrentScenario)
    try_message("UTC time:", mt.GetUTCTime)


def get_output_config(config_arg):
    """Parse the mark IV output configuration argument."""
    # code and max frequency
    code_dict = {
        'tt': (0x0810, 1),
        'iu': (0x1010, 2000),
        'ip': (0x1020, 2000),
        'ii': (0x1030, 2000),
        'if': (0x1060, 2000),
        'ic': (0x1070, 2000),
        'ir': (0x1080, 2000),
        'oq': (0x2010, 400),
        'om': (0x2020, 400),
        'oe': (0x2030, 400),
        'bp': (0x3010, 50),
        'ad': (0x4010, 2000),
        'aa': (0x4020, 2000),
        'af': (0x4030, 2000),
        'ah': (0x4040, 1000),
        'pa': (0x5020, 400),
        'pp': (0x5030, 400),
        'pl': (0x5040, 400),
        'np': (0x7010, 4),
        'ns': (0x7020, 4),
        'wr': (0x8020, 2000),
        'wd': (0x8030, 2000),
        'wh': (0x8040, 1000),
        'gd': (0x8830, 4),
        'gs': (0x8840, 4),
        'gu': (0x8880, 4),
        'gi': (0x88A0, 4),
        'rr': (0xA010, 2000),
        'rt': (0xA020, 2000),
        'mf': (0xC020, 100),
        'vv': (0xD010, 400),
        'sb': (0xE010, 2000),
        'sw': (0xE020, 2000)
    }
    # format flags
    format_dict = {'f': 0x00, 'd': 0x03, 'e': 0x00, 'n': 0x04, 'w': 0x08}
    config_re = re.compile('([a-z]{2})(\d+)?([fdenw])?([fdnew])?')
    output_configuration = []
    try:
        for item in config_arg.split(','):
            group, frequency, fmt1, fmt2 = config_re.findall(item.lower())[0]
            code, max_freq = code_dict[group]
            if fmt1 in format_dict:
                code |= format_dict[fmt1]
            if fmt2 in format_dict:
                code |= format_dict[fmt2]
            if frequency:
                frequency = min(max_freq, int(frequency))
            else:
                frequency = max_freq
            output_configuration.append((code, frequency))
        return output_configuration
    except (IndexError, KeyError):
        print('could not parse output specification "%s"' % item)
        return


def get_mode(arg):
    """Parse command line output-mode argument."""
    try:  # decimal
        mode = int(arg)
        return mode
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            mode = int(arg, 2)
            return mode
        except ValueError:
            pass
        try:  # hexadecimal
            mode = int(arg, 16)
            return mode
        except ValueError:
            pass
    # string mode specification
    mode = 0
    for c in arg:
        if c == 't':
            mode |= OutputMode.Temp
        elif c == 'c':
            mode |= OutputMode.Calib
        elif c == 'o':
            mode |= OutputMode.Orient
        elif c == 'a':
            mode |= OutputMode.Auxiliary
        elif c == 'p':
            mode |= OutputMode.Position
        elif c == 'v':
            mode |= OutputMode.Velocity
        elif c == 's':
            mode |= OutputMode.Status
        elif c == 'g':
            mode |= OutputMode.RAWGPS
        elif c == 'r':
            mode |= OutputMode.RAW
        else:
            print("Unknown output-mode specifier: '%s'" % c)
            return
    return mode


def get_settings(arg):
    """Parse command line output-settings argument."""
    try:  # decimal
        settings = int(arg)
        return settings
    except ValueError:
        pass
    if arg[0] == '0':
        try:  # binary
            settings = int(arg, 2)
            return settings
        except ValueError:
            pass
        try:  # hexadecimal
            settings = int(arg, 16)
            return settings
        except ValueError:
            pass
    # strings settings specification
    timestamp = 0
    orient_mode = 0
    calib_mode = OutputSettings.CalibMode_Mask
    NED = 0
    for c in arg:
        if c == 't':
            timestamp = OutputSettings.Timestamp_SampleCnt
        elif c == 'n':
            timestamp = OutputSettings.Timestamp_None
        elif c == 'u':
            timestamp |= OutputSettings.Timestamp_UTCTime
        elif c == 'q':
            orient_mode = OutputSettings.OrientMode_Quaternion
        elif c == 'e':
            orient_mode = OutputSettings.OrientMode_Euler
        elif c == 'm':
            orient_mode = OutputSettings.OrientMode_Matrix
        elif c == 'A':
            calib_mode &= OutputSettings.CalibMode_Acc
        elif c == 'G':
            calib_mode &= OutputSettings.CalibMode_Gyr
        elif c == 'M':
            calib_mode &= OutputSettings.CalibMode_Mag
        elif c == 'i':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN2
        elif c == 'j':
            calib_mode &= OutputSettings.AuxiliaryMode_NoAIN1
        elif c == 'N':
            NED = OutputSettings.Coordinates_NED
        else:
            print("Unknown output-settings specifier: '%s'" % c)
            return
    settings = timestamp | orient_mode | calib_mode | NED
    return settings


def get_synchronization_settings(arg):
    """Parse command line synchronization-settings argument."""
    if arg == "clear":
        sync_settings = [0, 0, 0, 0, 0, 0, 0, 0]
        return sync_settings
    else:
        # Parse each field from the argument
        sync_settings = arg.split(',')
        try:
            # convert string to int
            sync_settings = tuple([int(i) for i in sync_settings])
        except ValueError:
            print("Synchronization sync_settings must be integers.")
            return
        # check synchronization sync_settings
        if sync_settings[0] in (3, 4, 8, 9, 11) and \
                sync_settings[1] in (0, 1, 2, 4, 5, 6) and \
                sync_settings[2] in (1, 2, 3) and \
                sync_settings[3] in (0, 1):
            return sync_settings
        else:
            print("Invalid synchronization settings.")
            return


def get_UTCtime(arg):
    """Parse command line UTC time specification."""
    # If argument is now, fill the time settings with the current time
    # else fill the time settings with the specified time
    if arg == "now":
        timestamp = datetime.datetime.utcnow()  # use datetime to get microsec
        time_settings = []
        time_settings.append(timestamp.year)
        time_settings.append(timestamp.month)
        time_settings.append(timestamp.day)
        time_settings.append(timestamp.hour)
        time_settings.append(timestamp.minute)
        time_settings.append(timestamp.second)
        time_settings.append(timestamp.microsecond*1000)  # *1000 to get ns
        time_settings.append(0)  # default flag to 0
        return time_settings
    else:
        # Parse each field from the argument
        time_settings = arg.split(',')
        try:
            time_settings = [int(i) for i in time_settings]
        except ValueError:
            print("UTCtime settings must be integers.")
            return

        # check UTCtime settings
        if 1999 <= time_settings[0] <= 2099 and \
                1 <= time_settings[1] <= 12 and \
                1 <= time_settings[2] <= 31 and \
                0 <= time_settings[3] <= 23 and \
                0 <= time_settings[4] <= 59 and \
                0 <= time_settings[5] <= 59 and \
                0 <= time_settings[6] <= 1000000000:
            return time_settings
        else:
            print("Invalid UTCtime settings.")
            return


def get_gnss_platform(arg):
    """Parse and check command line GNSS platform argument."""
    try:
        platform = int(arg)
    except ValueError:
        print("GNSS platform must be an integer.")
        return
    if platform in (0, 8):
        return platform
    else:
        print("Invalid GNSS platform argument (excepted 0 or 8).")
        return


def get_option_flags(arg):
    """Parse and check command line option flags argument."""
    try:
        set_flag, clear_flag = map(lambda s: int(s.strip(), base=0),
                                   arg.split(','))
        return (set_flag, clear_flag)
    except ValueError:
        print('incorrect option flags specification (expected a pair of '
              'values)')
        return


def get_icc_command(arg):
    """Parse and check ICC command argument."""
    try:
        icc_command = int(arg, base=0)
        if icc_command not in range(4):
            raise ValueError
        return icc_command
    except ValueError:
        print('invalid ICC command "%s"; expected 0, 1, 2, or 3.' % arg)
        return


if __name__ == '__main__':
    main()
