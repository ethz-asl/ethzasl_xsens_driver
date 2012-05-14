"""Constant and messages definition for MT communication."""


class MID:
	"""Values for the message id (MID)"""
	## Error message, 1 data byte
	Error = 0x42
	ErrorCodes = {
		0x03: "Invalid period",
		0x04: "Invalid message",
		0x1E: "Timer overflow",
		0x20: "Invalid baudrate",
		0x21: "Invalid parameter"
	}

	# State MID
	## Switch to measurement state
	GoToMeasurement = 0x10
	## Switch to config state
	GoToConfig = 0x30
	## Reset device
	Reset = 0x40

	# Informational messages
	## Request device id
	ReqDID = 0x00
	## DeviceID, 4 bytes: HH HL LH LL
	DeviceID = 0x01
	## Request product code in plain text
	ReqProductCode = 0x1C
	## Product code (max 20 bytes data)
	ProductCode = 0x1d
	## Request firmware revision
	ReqFWRev = 0x12
	## Firmware revision, 3 bytes: major minor rev
	FirmwareRev = 0x13
	## Request data length according to current configuration
	ReqDataLength = 0x0a
	## Data Length, 2 bytes
	DataLength = 0x0b
	## Request GPS status
	ReqGPSStatus = 0xA6
	## GPS status
	GPSStatus = 0xA7

	# Device specific messages
	## Request baudrate
	ReqBaudrate = 0x18
	## Set next baudrate
	SetBaudrate = 0x18
	## Restore factory defaults
	RestoreFactoryDef = 0x0E

	# Configuration messages
	## Request configuration
	ReqConfiguration = 0x0C
	## Configuration, 118 bytes
	Configuration = 0x0D
	## Set sampling period, 2 bytes
	SetPeriod = 0x04
	## Set skip factor
	SetOutputSkipFactor = 0xD4
	## Set output mode, 2 bytes
	SetOutputMode = 0xD0
	## Set output settings, 4 bytes
	SetOutputSettings = 0xD2

	# Data messages
	## Data packet
	MTData = 0x32

	# XKF Filter messages
	## Request the available XKF scenarios on the device
	ReqAvailableScenarios = 0x62
	## Request the ID of the currently used scenario
	ReqCurrentScenario = 0x64
	## Set the scenario to use, 2 bytes
	SetCurrentScenario = 0x64


class Baudrates(object):
	"""Baudrate information and conversion."""
	## Baudrate mapping between ID and value
	Baudrates = [
		(0x00, 460800),
		(0x01, 230400),
		(0x02, 115200),
		(0x03,  76800),
		(0x04,  57600),
		(0x05,  38400),
		(0x06,  28800),
		(0x07,  19200),
		(0x08,  14400),
		(0x09,   9600),
		(0x80, 921600)]
	@classmethod
	def get_BRID(cls, baudrate):
		"""Get baudrate id for a given baudrate."""
		for brid, br in cls.Baudrates:
			if baudrate==br:
				return brid
		raise MTException("unsupported baudrate.")
	@classmethod
	def	get_BR(cls, baudrate_id):
		"""Get baudrate for a given baudrate id."""
		for brid, br in cls.Baudrates:
			if baudrate_id==brid:
				return br
		raise MTException("unknown baudrate id.")
	

class OutputMode:
	"""Values for the output mode."""
	Temp 		= 0x0001
	Calib 		= 0x0002
	Orient 		= 0x0004
	Auxiliary 	= 0x0008
	Position 	= 0x0010
	Velocity 	= 0x0020
	Status 		= 0x0800
	RAWGPS 		= 0x1000	# supposed to be incompatible with previous
	RAW 		= 0x4000	# incompatible with all except RAWGPS


class OutputSettings:
	"""Values for the output settings."""
	Timestamp_None			= 0x00000000
	Timestamp_SampleCnt 	= 0x00000001
	OrientMode_Quaternion 	= 0x00000000
	OrientMode_Euler		= 0x00000004
	OrientMode_Matrix		= 0x00000008
	CalibMode_AccGyrMag 	= 0x00000000
	CalibMode_GyrMag 		= 0x00000010
	CalibMode_AccMag 		= 0x00000020
	CalibMode_Mag 			= 0x00000030
	CalibMode_AccGyr 		= 0x00000040
	CalibMode_Gyr 			= 0x00000050
	CalibMode_Acc 			= 0x00000060
	CalibMode_Mask 			= 0x00000070
	DataFormat_Float 		= 0x00000000
	DataFormat_12_20 		= 0x00000100	# not supported yet
	DataFormat_16_32 		= 0x00000200	# not supported yet
	DataFormat_Double 		= 0x00000300	# not supported yet
	AuxiliaryMode_NoAIN1 	= 0x00000400
	AuxiliaryMode_NoAIN2 	= 0x00000800
	PositionMode_LLA_WGS84 	= 0x00000000
	VelocityMode_MS_XYZ 	= 0x00000000
	Coordinates_NED 		= 0x80000000


class MTException(Exception):
	def __init__(self, message):
		self.message = message
	def __str__(self):
		return "MT error: " + self.message
