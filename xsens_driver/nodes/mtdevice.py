#!/usr/bin/env python
import serial
import struct

import sys, getopt, time, glob#, traceback

from mtdef import MID, OutputMode, OutputSettings, MTException, Baudrates




################################################################
# MTDevice class
################################################################
## XSens MT device communication object.
class MTDevice(object):
	"""XSens MT device communication object."""

	def __init__(self, port, baudrate=115200, timeout=0.1, autoconf=True,
			config_mode=False):
		"""Open device."""
		## serial interface to the device
		self.device = serial.Serial(port, baudrate, timeout=timeout,
				writeTimeout=timeout)
		self.device.flushInput()	# flush to make sure the port is ready TODO
		self.device.flushOutput()	# flush to make sure the port is ready TODO
		## timeout for communication
		self.timeout = timeout
		if autoconf:
			self.auto_config()
		else:
			## mode parameter of the IMU
			self.mode = None
			## settings parameter of the IMU
			self.settings = None
			## length of the MTData message
			self.length = None
			## header of the MTData message
			self.header = None
		if config_mode:
			self.GoToConfig()

	############################################################
	# Low-level communication
	############################################################
	
	## Low-level message sending function.
	def write_msg(self, mid, data=[]):
		"""Low-level message sending function."""
		length = len(data)
		if length>254:
			lendat = [0xFF, 0xFF&length, 0xFF&(length>>8)]
		else:
			lendat = [length]
		packet = [0xFA, 0xFF, mid] + lendat + list(data)
		packet.append(0xFF&(-(sum(packet[1:]))))
		msg = struct.pack('%dB'%len(packet), *packet)
		self.device.write(msg)
#		print "MT: Write message id 0x%02X with %d data bytes: [%s]"%(mid,length,
#				' '.join("%02X"% v for v in data))
		#self.device.flush() #TODO evaluate

	## Low-level MTData receiving function.
	# Take advantage of known message length.
	def read_data_msg(self, buf=bytearray()):
		"""Low-level MTData receiving function.
		Take advantage of known message length."""
		start = time.time()
		if self.length>254:
			totlength = 7 + self.length
		else:
			totlength = 5 + self.length
		while (time.time()-start)<self.timeout:
			while len(buf)<totlength:
				buf.extend(self.device.read(totlength-len(buf)))
			preamble_ind = buf.find(self.header)
			if preamble_ind==-1:	# not found
				# discard unexploitable data
				#sys.stderr.write("MT: discarding (no preamble).\n")
				del buf[:-3]
				continue
			elif preamble_ind:	# found but not at start
				# discard leading bytes
				#sys.stderr.write("MT: discarding (before preamble).\n")
				del buf[:preamble_ind]
				# complete message for checksum
				while len(buf)<totlength:
					buf.extend(self.device.read(totlength-len(buf)))
			if 0xFF&sum(buf[1:]):
				#sys.stderr.write("MT: invalid checksum; discarding data and "\
				#		"waiting for next message.\n")
				del buf[:buf.find(self.header)-2]
				continue
			data = str(buf[-self.length-1:-1])
			del buf[:]
			return data
		else:
			raise MTException("could not find MTData message.")



	## Low-level message receiving function.
	def read_msg(self):
		"""Low-level message receiving function."""
		start = time.time()
		while (time.time()-start)<self.timeout:
			# read first char of preamble
			c = self.device.read()
			if not c:
				raise MTException("timeout waiting for message.")
			if ord(c)<>0xFA:
				continue
			# second part of preamble
			if ord(self.device.read())<>0xFF:	# we assume no timeout anymore
				continue
			# read message id and length of message
			mid, length = struct.unpack('!BB', self.device.read(2))
			if length==255:	# extended length
				length, = struct.unpack('!H', self.device.read(2))
			# read contents and checksum
			buf = self.device.read(length+1)
			while (len(buf)<length+1) and ((time.time()-start)<self.timeout):
				buf+= self.device.read(length+1-len(buf))
			if (len(buf)<length+1):
				continue
			checksum = ord(buf[-1])
			data = struct.unpack('!%dB'%length, buf[:-1])
			if mid == MID.Error:
				sys.stderr.write("MT error 0x%02X: %s."%(data[0],
						MID.ErrorCodes[data[0]]))
#			print "MT: Got message id 0x%02X with %d data bytes: [%s]"%(mid,length,
#					' '.join("%02X"% v for v in data))
			if 0xFF&sum(data, 0xFF+mid+length+checksum):
				sys.stderr.write("invalid checksum; discarding data and "\
						"waiting for next message.\n")
				continue
			return (mid, buf[:-1])
		else:
			raise MTException("could not find message.")

	## Send a message and read confirmation
	def write_ack(self, mid, data=[]):
		"""Send a message a read confirmation."""
		self.write_msg(mid, data)
		for tries in range(10):
			mid_ack, data_ack = self.read_msg()
			if mid_ack==(mid+1):
				break
		else:
			raise MTException("Ack (0x%X) expected, MID 0x%X received instead"\
					" (after 10 tries)."%(mid+1, mid_ack))
		return data_ack

		

	############################################################
	# High-level functions
	############################################################
	## Reset MT device.
	def Reset(self):
		"""Reset MT device."""
		self.write_ack(MID.Reset)


	## Place MT device in configuration mode.
	def GoToConfig(self):
		"""Place MT device in configuration mode."""
		self.write_ack(MID.GoToConfig)


	## Place MT device in measurement mode.
	def GoToMeasurement(self):
		"""Place MT device in measurement mode."""
		self.write_ack(MID.GoToMeasurement)


	## Restore MT device configuration to factory defaults (soft version).
	def RestoreFactoryDefaults(self):
		"""Restore MT device configuration to factory defaults (soft version).
		"""
		self.GoToConfig()
		self.write_ack(MID.RestoreFactoryDef)


	## Get current output mode.
	# Assume the device is in Config state.
	def GetOutputMode(self):
		"""Get current output mode.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetOutputMode)
		self.mode, = struct.unpack('!H', data)
		return self.mode

	
	## Select which information to output.
	# Assume the device is in Config state.
	def SetOutputMode(self, mode):
		"""Select which information to output.
		Assume the device is in Config state."""
		H, L = (mode&0xFF00)>>8, mode&0x00FF
		self.write_ack(MID.SetOutputMode, (H, L))

	
	## Get current output mode.
	# Assume the device is in Config state.
	def GetOutputSettings(self):
		"""Get current output mode.
		Assume the device is in Config state."""
		data = self.write_ack(MID.SetOutputSettings)
		self.settings, = struct.unpack('!I', data)
		return self.settings

	
	## Select how to output the information.
	# Assume the device is in Config state.
	def SetOutputSettings(self, settings):
		"""Select how to output the information.
		Assume the device is in Config state."""
		HH, HL = (settings&0xFF000000)>>24, (settings&0x00FF0000)>>16
		LH, LL = (settings&0x0000FF00)>>8, settings&0x000000FF
		self.write_ack(MID.SetOutputSettings, (HH, HL, LH, LL))
	

	## Set the period of sampling.
	# Assume the device is in Config state.
	def SetPeriod(self, period):
		"""Set the period of sampling.
		Assume the device is in Config state."""
		H, L = (period&0xFF00)>>8, period&0x00FF
		self.write_ack(MID.SetPeriod, (H, L))


	## Set the output skip factor.
	# Assume the device is in Config state.
	def SetOutputSkipFactor(self, skipfactor):
		"""Set the output skip factor.
		Assume the device is in Config state."""
		H, L = (skipfactor&0xFF00)>>8, skipfactor&0x00FF
		self.write_ack(MID.SetOutputSkipFactor, (H, L))


	## Get data length.
	# Assume the device is in Config state.
	def ReqDataLength(self):
		"""Get data length.
		Assume the device is in Config state."""
		data = self.write_ack(MID.ReqDataLength)
		self.length, = struct.unpack('!H', data)
		self.header = '\xFA\xFF\x32'+chr(self.length)
		return self.length

	
	## Ask for the current configuration of the MT device.
	# Assume the device is in Config state.
	def ReqConfiguration(self):
		"""Ask for the current configuration of the MT device.
		Assume the device is in Config state."""
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
		self.header = '\xFA\xFF\x32'+chr(length)
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
	

	## Set the baudrate of the device using the baudrate id.
	# Assume the device is in Config state.
	def SetBaudrate(self, brid):
		"""Set the baudrate of the device using the baudrate id.
		Assume the device is in Config state."""
		self.write_ack(MID.SetBaudrate, (brid,))


	## Request the available XKF scenarios on the device.
	# Assume the device is in Config state.
	def ReqAvailableScenarios(self):
		"""Request the available XKF scenarios on the device.
		Assume the device is in Config state."""
		scenarios_dat = self.write_ack(MID.ReqAvailableScenarios)
		scenarios = []
		try:
			for i in range(len(scenarios_dat)/22):
				scenario_type, version, label =\
						struct.unpack('!BB20s', scenarios_dat[22*i:22*(i+1)])
				scenarios.append((scenario_type, version, label.strip()))
			## available XKF scenarios
			self.scenarios = scenarios
		except struct.error:
			raise MTException("could not parse the available XKF scenarios.")
		return scenarios


	## Request the ID of the currently used XKF scenario.
	# Assume the device is in Config state.
	def ReqCurrentScenario(self):
		"""Request the ID of the currently used XKF scenario.
		Assume the device is in Config state."""
		data = self.write_ack(MID.ReqCurrentScenario)
		## current XKF id
		self.scenario_id, = struct.unpack('!H', data)
		try:
			scenarios = self.scenarios
		except AttributeError:
			scenarios = self.ReqAvailableScenarios()
		for t, _, label in scenarios:
			if t==self.scenario_id:
				## current XKF label
				self.scenario_label = label
				break
		else:
			self.scenario_label = ""
		return self.scenario_id, self.scenario_label


	## Sets the XKF scenario to use.
	# Assume the device is in Config state.
	def SetCurrentScenario(self, scenario_id):
		"""Sets the XKF scenario to use.
		Assume the device is in Config state."""
		self.write_ack(MID.SetCurrentScenario, (0x00, scenario_id&0xFF))


	############################################################
	# High-level utility functions
	############################################################
	## Configure the mode and settings of the MT device.
	def configure(self, mode, settings, period=None, skipfactor=None):
		"""Configure the mode and settings of the MT device."""
		self.GoToConfig()
		self.SetOutputMode(mode)
		self.SetOutputSettings(settings)
		if period is not None:
			self.SetPeriod(period)
		if skipfactor is not None:
			self.SetOutputSkipFactor(skipfactor)

		self.GetOutputMode()
		self.GetOutputSettings()
		self.ReqDataLength()
		self.GoToMeasurement()


	## Read configuration from device.
	def auto_config(self):
		"""Read configuration from device."""
		self.GoToConfig()
		mode = self.GetOutputMode()
		settings = self.GetOutputSettings()
		length = self.ReqDataLength()
		self.GoToMeasurement()
		return mode, settings, length

	## Read and parse a measurement packet
	def read_measurement(self, mode=None, settings=None):
		"""Read and parse a measurement packet."""
		# getting mode
		if mode is None:
			mode = self.mode
		if settings is None:
			settings = self.settings
		# getting data
		data = self.read_data_msg()
		#_, data = self.read_msg()
		# data object
		output = {}
		try:
			# raw IMU first
			if mode & OutputMode.RAW:
				o = {}
				o['accX'], o['accY'], o['accZ'], o['gyrX'], o['gyrY'], o['gyrZ'],\
						o['magX'], o['magY'], o['magZ'], o['temp'] =\
						struct.unpack('!10H', data[:20])
				data = data[20:]
				output['RAW'] = o
			# raw GPS second
			if mode & OutputMode.RAWGPS:
				o = {}
				o['Press'], o['bPrs'], o['ITOW'], o['LAT'], o['LON'], o['ALT'],\
						o['VEL_N'], o['VEL_E'], o['VEL_D'], o['Hacc'], o['Vacc'],\
						o['Sacc'], o['bGPS'] = struct.unpack('!HBI6i3IB', data[:44])
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
				if not (settings&OutputSettings.CalibMode_GyrMag):
					o['accX'], o['accY'], o['accZ'] = struct.unpack('!3f',\
							 data[:12])
					data = data[12:]
				if not (settings&OutputSettings.CalibMode_AccMag):
					o['gyrX'], o['gyrY'], o['gyrZ'] = struct.unpack('!3f',\
							 data[:12])
					data = data[12:]
				if not (settings&OutputSettings.CalibMode_AccGyr):
					o['magX'], o['magY'], o['magZ'] = struct.unpack('!3f',\
							 data[:12])
					data = data[12:]
				output['Calib'] = o
			# orientation
			if mode & OutputMode.Orient:
				o = {}
				if settings & OutputSettings.OrientMode_Euler:
					o['roll'], o['pitch'], o['yaw'] = struct.unpack('!3f', data[:12])
					data = data[12:]
				elif settings & OutputSettings.OrientMode_Matrix:
					a, b, c, d, e, f, g, h, i = struct.unpack('!9f', data[:36])
					data = data[36:]
					o['matrix'] = ((a, b, c), (d, e, f), (g, h, i))
				else: # OutputSettings.OrientMode_Quaternion:
					q0, q1, q2, q3 = struct.unpack('!4f', data[:16])
					data = data[16:]
					o['quaternion'] = (q0, q1, q2, q3)
				output['Orient'] = o
			# auxiliary
			if mode & OutputMode.Auxiliary:
				o = {}
				if not (settings&OutputSettings.AuxiliaryMode_NoAIN1):
					o['Ain_1'], = struct.unpack('!H', data[:2])
					data = data[2:]
				if not (settings&OutputSettings.AuxiliaryMode_NoAIN2):
					o['Ain_2'], = struct.unpack('!H', data[:2])
					data = data[2:]
				output['Auxiliary'] = o
			# position
			if mode & OutputMode.Position:
				o = {}
				o['Lat'], o['Lon'], o['Alt'] = struct.unpack('!3f', data[:12])
				data = data[12:]
				output['Position'] = o
			# velocity
			if mode & OutputMode.Velocity:
				o = {}
				o['Vel_X'], o['Vel_Y'], o['Vel_Z'] = struct.unpack('!3f', data[:12])
				data = data[12:]
				output['Velocity'] = o
			# status
			if mode & OutputMode.Status:
				status, = struct.unpack('!B', data[:1])
				data = data[1:]
				output['Status'] = status
			# sample counter
			if settings & OutputSettings.Timestamp_SampleCnt:
				TS, = struct.unpack('!H', data[:2])
				data = data[2:]
				output['Sample'] = TS
		except struct.error, e:
			raise MTException("could not parse MTData message.")
		if data <> '':
			raise MTException("could not parse MTData message (too long).")
		return output


	## Change the baudrate, reset the device and reopen communication.
	def ChangeBaudrate(self, baudrate):
		"""Change the baudrate, reset the device and reopen communication."""
		self.GoToConfig()
		brid = Baudrates.get_BRID(baudrate)
		self.SetBaudrate(brid)
		self.Reset()
		#self.device.flush()
		self.device.baudrate=baudrate
		#self.device.flush()
		time.sleep(0.01)
		self.read_msg()
		self.write_msg(0x3f)




################################################################
# Auto detect port
################################################################
def find_devices():
	mtdev_list = []
	for port in glob.glob("/dev/tty*S*"):
		try:
			br = find_baudrate(port)
			if br:
				mtdev_list.append((port, br))
		except MTException:
			pass
	return mtdev_list


################################################################
# Auto detect baudrate
################################################################
def find_baudrate(port):
	baudrates = (115200, 460800, 921600, 230400, 57600, 38400, 19200, 9600)
	for br in baudrates:
		try:
			mt = MTDevice(port, br)
		except serial.SerialException:
			raise MTException("unable to open %s"%port)
		try:
			mt.GoToConfig()
			mt.GoToMeasurement()
			return br
		except MTException:
			pass



################################################################
# Documentation for stand alone usage
################################################################
def usage():
		print """MT device driver.
Usage:
	./mtdevice.py [commands] [opts]

Commands:
	-h, --help
		Print this help and quit.
	-r, --reset
		Reset device to factory defaults.
	-a, --change-baudrate=NEW_BAUD
		Change baudrate from BAUD (see below) to NEW_BAUD.
	-c, --configure
		Configure the device (needs MODE and SETTINGS arguments below).
	-e, --echo
		Print MTData. It is the default if no other command is supplied.
	-i, --inspect
		Print current MT device configuration.
	-x, --xkf-scenario=ID
		Change the current XKF scenario.
		

Options:
	-d, --device=DEV
		Serial interface of the device (default: /dev/ttyUSB0). If 'auto', then
		all serial ports are tested at all baudrates and the first
		suitable device is used.
	-b, --baudrate=BAUD
		Baudrate of serial interface (default: 115200). If 0, then all
		rates are tried until a suitable one is found.
	-m, --output-mode=MODE
		Mode of the device selecting the information to output.
		This is required for 'configure' command. If it is not present
		in 'echo' command, the configuration will be read from the
		device.
		MODE can be either the mode value in hexadecimal, decimal or
		binary form, or a string composed of the following characters
		(in any order):
			t	temperature, [0x0001]
			c	calibrated data, [0x0002]
			o	orientation data, [0x0004]
			a	auxiliary data, [0x0008]
			p	position data (requires MTi-G), [0x0010]
			v	velocity data (requires MTi-G), [0x0020]
			s	status data, [0x0800]
			g	raw GPS mode (requires MTi-G), [0x1000]
			r	raw (incompatible with others except raw GPS),
				[0x4000]
		For example, use "--output-mode=so" to have status and
		orientation data.
	-s, --output-settings=SETTINGS
		Settings of the device.
		This is required for 'configure' command. If it is not present
		in 'echo' command, the configuration will be read from the
		device.
		SETTINGS can be either the settings value in hexadecimal,
		decimal or binary form, or a string composed of the following
		characters (in any order):
			t	sample count (excludes 'n')
			n	no sample count (excludes 't')
			q	orientation in quaternion (excludes 'e' and 'm')
			e	orientation in Euler angles (excludes 'm' and
				'q')
			m	orientation in matrix (excludes 'q' and 'e')
			A	acceleration in calibrated data
			G	rate of turn in calibrated data
			M	magnetic field in calibrated data
			i	only analog input 1 (excludes 'j')
			j	only analog input 2 (excludes 'i')
			N	North-East-Down instead of default: X North Z up
		For example, use "--output-settings=tqMAG" for all calibrated
		data, sample counter and orientation in quaternion.
	-p, --period=PERIOD
		Sampling period in (1/115200) seconds (default: 1152).
		Minimum is 225 (1.95 ms, 512 Hz), maximum is 1152
		(10.0 ms, 100 Hz).
		Note that it is the period at which sampling occurs, not the
		period at which messages are sent (see below).
	-f, --skip-factor=SKIPFACTOR
		Number of samples to skip before sending MTData message
		(default: 0).
		The frequency at which MTData message is send is:
			115200/(PERIOD * (SKIPFACTOR + 1))
		If the value is 0xffff, no data is send unless a ReqData request
		is made.
"""


################################################################
# Main function
################################################################
def main():
	# parse command line
	shopts = 'hra:ceid:b:m:s:p:f:x:'
	lopts = ['help', 'reset', 'change-baudrate=', 'configure', 'echo',
			'inspect', 'device=', 'baudrate=', 'output-mode=',
			'output-settings=', 'period=', 'skip-factor=', 'xkf-scenario=']
	try:
		opts, args = getopt.gnu_getopt(sys.argv[1:], shopts, lopts)
	except getopt.GetoptError, e:
		print e
		usage()
		return 1
	# default values
	device = '/dev/ttyUSB0'
	baudrate = 115200
	mode = None
	settings = None
	period = None
	skipfactor = None
	new_baudrate = None
	new_xkf = None
	actions = []
	# filling in arguments
	for o, a in opts:
		if o in ('-h', '--help'):
			usage()
			return
		if o in ('-r', '--reset'):
			actions.append('reset')
		if o in ('-a', '--change-baudrate'):
			try:
				new_baudrate = int(a)
			except ValueError:
				print "change-baudrate argument must be integer."
				return 1
			actions.append('change-baudrate')
		if o in ('-c', '--configure'):
			actions.append('configure')
		if o in ('-e', '--echo'):
			actions.append('echo')
		if o in ('-i', '--inspect'):
			actions.append('inspect')
		if o in ('-x', '--xkf-scenario'):
			try:
				new_xkf = int(a)
			except ValueError:
				print "xkf-scenario argument must be integer."
				return 1
			actions.append('xkf-scenario')
		if o in ('-d', '--device'):
			device = a
		if o in ('-b', '--baudrate'):
			try:
				baudrate = int(a)
			except ValueError:
				print "baudrate argument must be integer."
				return 1
		if o in ('-m', '--output-mode'):
			mode = get_mode(a)
			if mode is None:
				return 1
		if o in ('-s', '--output-settings'):
			settings = get_settings(a)
			if settings is None:
				return 1
		if o in ('-p', '--period'):
			try:
				period = int(a)
			except ValueError:
				print "period argument must be integer."
				return 1
		if o in ('-f', '--skip-factor'):
			try:
				skipfactor = int(a)
			except ValueError:
				print "skip-factor argument must be integer."
				return 1
	# if nothing else: echo 
	if len(actions) == 0:
		actions.append('echo')
	try:
		if device=='auto':
			devs = find_devices()
			if devs:
				print "Detected devices:","".join('\n\t%s @ %d'%(d,p) for d,p in
						devs)
				print "Using %s @ %d"%devs[0]
				device, baudrate = devs[0]
			else:
				print "No suitable device found."
				return 1
		# find baudrate
		if not baudrate:
			baudrate = find_baudrate(device)
		if not baudrate:
			print "No suitable baudrate found."
			return 1
		# open device
		try:
			mt = MTDevice(device, baudrate)
		except serial.SerialException:
			raise MTException("unable to open %s"%device)
		# execute actions
		if 'inspect' in actions:
			mt.GoToConfig()
			print "Device: %s at %d Bd:"%(device, baudrate) 
			print "General configuration:", mt.ReqConfiguration()
			print "Available scenarios:", mt.ReqAvailableScenarios()
			print "Current scenario: %s (id: %d)"%mt.ReqCurrentScenario()[::-1]
			mt.GoToMeasurement()
		if 'change-baudrate' in actions:
			print "Changing baudrate from %d to %d:"%(baudrate, new_baudrate),
			sys.stdout.flush()
			mt.ChangeBaudrate(new_baudrate)
			print " Ok"		# should we test it was actually ok?
		if 'reset' in actions:
			print "Restoring factory defaults",
			sys.stdout.flush()
			mt.RestoreFactoryDefaults()
			print " Ok"		# should we test it was actually ok?
		if 'configure' in actions:
			if mode is None:
				print "output-mode is require to configure the device."
				return 1
			if settings is None:
				print "output-settings is required to configure the device."
				return 1
			print "Configuring mode and settings",
			sys.stdout.flush()
			mt.configure(mode, settings, period, skipfactor)
			print " Ok"		# should we test it was actually ok?
		if 'xkf-scenario' in actions:
			print "Changing XKF scenario",
			sys.stdout.flush()
			mt.GoToConfig()
			mt.SetCurrentScenario(new_xkf)
			mt.GoToMeasurement()
			print "Ok"
		if 'echo' in actions:
#			if (mode is None) or (settings is None):
#				mode, settings, length = mt.auto_config()
#				print mode, settings, length
			try:
				while True:
					print mt.read_measurement(mode, settings)
			except KeyboardInterrupt:
				pass
	except MTException as e:
		#traceback.print_tb(sys.exc_info()[2])
		print e
		


def get_mode(arg):
	"""Parse command line output-mode argument."""
	try:	# decimal
		mode = int(arg)
		return mode
	except ValueError:
		pass
	if arg[0]=='0':
		try:	# binary
			mode = int(arg, 2)
			return mode
		except ValueError:
			pass
		try:	# hexadecimal
			mode = int(arg, 16)
			return mode
		except ValueError:
			pass
	# string mode specification
	mode = 0
	for c in arg:
		if c=='t':
			mode |= OutputMode.Temp
		elif c=='c':
			mode |= OutputMode.Calib
		elif c=='o':
			mode |= OutputMode.Orient
		elif c=='a':
			mode |= OutputMode.Auxiliary
		elif c=='p':
			mode |= OutputMode.Position
		elif c=='v':
			mode |= OutputMode.Velocity
		elif c=='s':
			mode |= OutputMode.Status
		elif c=='g':
			mode |= OutputMode.RAWGPS
		elif c=='r':
			mode |= OutputMode.RAW
		else:
			print "Unknown output-mode specifier: '%s'"%c
			return
	return mode

def get_settings(arg):
	"""Parse command line output-settings argument."""
	try:	# decimal
		settings = int(arg)
		return settings
	except ValueError:
		pass
	if arg[0]=='0':
		try:	# binary
			settings = int(arg, 2)
			return settings
		except ValueError:
			pass
		try:	# hexadecimal
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
		if c=='t':
			timestamp = OutputSettings.Timestamp_SampleCnt
		elif c=='n':
			timestamp = OutputSettings.Timestamp_None
		elif c=='q':
			orient_mode = OutputSettings.OrientMode_Quaternion
		elif c=='e':
			orient_mode = OutputSettings.OrientMode_Euler
		elif c=='m':
			orient_mode = OutputSettings.OrientMode_Matrix
		elif c=='A':
			calib_mode &= OutputSettings.CalibMode_Acc
		elif c=='G':
			calib_mode &= OutputSettings.CalibMode_Gyr
		elif c=='M':
			calib_mode &= OutputSettings.CalibMode_Mag
		elif c=='i':
			calib_mode &= OutputSettings.AuxiliaryMode_NoAIN2
		elif c=='j':
			calib_mode &= OutputSettings.AuxiliaryMode_NoAIN1
		elif c=='N':
			NED = OutputSettings.Coordinates_NED
		else:
			print "Unknown output-settings specifier: '%s'"%c
			return
	settings = timestamp|orient_mode|calib_mode|NED
	return settings


if __name__=='__main__':
	main()

