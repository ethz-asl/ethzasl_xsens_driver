import mtdevice
from mtdef import MID,MTException
import serial 
import struct
import sys
import time
import argparse
import os

def determine_file_name(name, startValue):
    """
    Determines the next valid file name build from prefix + INTEGER with length integer_length + extension
    example: determine_file_name('test'_', 3, '.bin') results in test_001.bin, test_002.bin, ...
    """
    nextFileName = name % startValue
    while os.path.isfile(nextFileName):
        startValue += 1
        nextFileName = name % startValue
    return nextFileName
    
def determine_file_name1(prefix, integer_length, extension):
    """
    Determines the next valid file name build from prefix + INTEGER with length integer_length + extension
    example: determine_file_name('test'_', 3, '.bin') results in test_001.bin, test_002.bin, ...
    """
    intMask = "%%0%dd" % integer_length
    print(intMask)
    index = 1
    
    nextFileName = prefix + (intMask % index) + extension
    while os.path.isfile(nextFileName):
        index += 1
        nextFileName = prefix + (intMask % index) + extension
    return nextFileName    
def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('-d', '--device', required=True)
    parser.add_argument('-b', '--baudrate', default=115200)
    args = parser.parse_args()
    if args.verbose:
        mtdevice.verbose = True    
    try:
        try:
            mt = mtdevice.MTDevice(args.device, args.baudrate)
        except serial.SerialException:
            raise MTException("unable to open serial port %s!"%args.device)
        print("Configuring calibration mode and settings")
        # configure mode and settings for required data
        mode = mtdevice.get_mode('r')
        settings = mtdevice.get_settings('t')
        mt.GoToConfig()
        mt.SetOutputMode(mode)
        mt.SetOutputSettings(settings)
        mt.SetPeriod(1152)
        mt.SetOutputSkipFactor(0)
        # read config data to get device ID
        conf = mt.ReqConfiguration()
        # write data to file
        fname = determine_file_name('MT_%08X_' % (conf['device ID']) + '%04d.mtb', 1)
        print("Writing to file %s. Press CTRL-C to stop logging." % fname)
        with open(fname, 'wb') as f:
            try:
                mt.set_logfile(f)
                # increase of timeout may be needed 
                mt.device.timeout = 0.1
                mt.ReqConfiguration()
                mt.ReqEmts()
                mt.GoToMeasurement()
                while True:
                    mt.read_msg()
            except KeyboardInterrupt:
                mt.set_logfile(None)
                f.close()
                pass
    except MTException as e:
        #traceback.print_tb(sys.exc_info()[2])
        print e
        
if __name__=='__main__':
    main()
