#!/usr/bin/env python
import roslib; roslib.load_manifest('xsens_driver')
import rospy
import select

import mtdevice

from std_msgs.msg import Header, Float32, String, UInt16
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from gps_common.msg import GPSFix, GPSStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time


# transform Euler angles or matrix into quaternions
from math import pi, radians
from tf.transformations import quaternion_from_matrix, quaternion_from_euler, identity_matrix


def get_param(name, default):
	try:
		v = rospy.get_param(name)
		rospy.loginfo("Found parameter: %s, value: %s"%(name, str(v)))
	except KeyError:
		v = default
		rospy.logwarn("Cannot find value for parameter: %s, assigning "
				"default: %s"%(name, str(v)))
	return v

class XSensDriver(object):
	
	def __init__(self):
		
		device = get_param('~device', 'auto')
		baudrate = get_param('~baudrate', 0)
		if device=='auto':
			devs = mtdevice.find_devices()
			if devs:
				device, baudrate = devs[0]
				rospy.loginfo("Detected MT device on port %s @ %d bps"%(device,
						baudrate))
			else:
				rospy.logerr("Fatal: could not find proper MT device.")
				rospy.signal_shutdown("Could not find proper MT device.")
				return
		if not baudrate:
			baudrate = mtdevice.find_baudrate(device)
		if not baudrate:
			rospy.logerr("Fatal: could not find proper baudrate.")
			rospy.signal_shutdown("Could not find proper baudrate.")
			return

		rospy.loginfo("MT node interface: %s at %d bd."%(device, baudrate))
		self.mt = mtdevice.MTDevice(device, baudrate)

		self.frame_id = get_param('~frame_id', '/base_imu')
		
		self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
		self.diag_msg = DiagnosticArray()
		self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1,
				message='No status information')
		self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1,
				message='No status information')
		self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1,
				message='No status information')
		self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]

		self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
		self.gps_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
		self.xgps_pub = rospy.Publisher('fix_extended', GPSFix, queue_size=10)
		self.vel_pub = rospy.Publisher('velocity', TwistStamped, queue_size=10)
		self.mag_pub = rospy.Publisher('magnetic', Vector3Stamped, queue_size=10)
		self.temp_pub = rospy.Publisher('temperature', Float32, queue_size=10)	# decide type+header
		self.press_pub = rospy.Publisher('pressure', Float32, queue_size=10) # decide type+header
		self.analog_in1_pub = rospy.Publisher('analog_in1', UInt16, queue_size=10) # decide type+header
		self.analog_in2_pub = rospy.Publisher('analog_in2', UInt16, queue_size=10) # decide type+header
		# TODO pressure, ITOW from raw GPS?
		self.old_bGPS = 256	# publish GPS only if new

		# publish a string version of all data; to be parsed by clients
		self.str_pub = rospy.Publisher('imu_data_str', String, queue_size=10)

	def reset_vars(self):
		self.imu_msg = Imu()
		self.imu_msg.orientation_covariance = (-1., )*9
		self.imu_msg.angular_velocity_covariance = (-1., )*9
		self.imu_msg.linear_acceleration_covariance = (-1., )*9
		self.pub_imu = False
		self.gps_msg = NavSatFix()
		self.xgps_msg = GPSFix()
		self.pub_gps = False
		self.vel_msg = TwistStamped()
		self.pub_vel = False
		self.mag_msg = Vector3Stamped()
		self.pub_mag = False
		self.temp_msg = Float32()
		self.pub_temp = False
		self.press_msg = Float32()
		self.pub_press = False
		self.anin1_msg = UInt16()
		self.pub_anin1 = False
		self.anin2_msg = UInt16()
		self.pub_anin2 = False
		self.pub_diag = False

	def spin(self):
		try:
			while not rospy.is_shutdown():
				self.spin_once()
				self.reset_vars()
		# Ctrl-C signal interferes with select with the ROS signal handler
		# should be OSError in python 3.?
		except select.error:
			pass

	def spin_once(self):
		'''Read data from device and publishes ROS messages.'''
		# common header
		self.h = Header()
		self.h.stamp = rospy.Time.now()
		self.h.frame_id = self.frame_id
		
		# set default values
		self.reset_vars()
		
		def fill_from_raw(raw_data):
			'''Fill messages with information from 'raw' MTData block.'''
			# don't publish raw imu data anymore
			# TODO find what to do with that
			pass
		
		def fill_from_rawgps(rawgps_data):
			'''Fill messages with information from 'rawgps' MTData block.'''
			if rawgps_data['bGPS']<self.old_bGPS:
				self.pub_gps = True
				# LLA
				self.xgps_msg.latitude = self.gps_msg.latitude = rawgps_data['LAT']*1e-7
				self.xgps_msg.longitude = self.gps_msg.longitude = rawgps_data['LON']*1e-7
				self.xgps_msg.altitude = self.gps_msg.altitude = rawgps_data['ALT']*1e-3
				# NED vel # TODO?
				# Accuracy
				# 2 is there to go from std_dev to 95% interval
				self.xgps_msg.err_horz = 2*rawgps_data['Hacc']*1e-3
				self.xgps_msg.err_vert = 2*rawgps_data['Vacc']*1e-3
			self.old_bGPS = rawgps_data['bGPS']
		
		def fill_from_Temp(temp):
			'''Fill messages with information from 'temperature' MTData block.'''
			self.pub_temp = True
			self.temp_msg.data = temp
		
		def fill_from_Calib(imu_data):
			'''Fill messages with information from 'calibrated' MTData block.'''
			try:
				self.pub_imu = True
				self.imu_msg.angular_velocity.x = imu_data['gyrX']
				self.imu_msg.angular_velocity.y = imu_data['gyrY']
				self.imu_msg.angular_velocity.z = imu_data['gyrZ']
				self.imu_msg.angular_velocity_covariance = (radians(0.025), 0., 0., 0.,
						radians(0.025), 0., 0., 0., radians(0.025))
				self.pub_vel = True
				self.vel_msg.twist.angular.x = imu_data['gyrX']
				self.vel_msg.twist.angular.y = imu_data['gyrY']
				self.vel_msg.twist.angular.z = imu_data['gyrZ']
			except KeyError:
				pass
			try:
				self.pub_imu = True
				self.imu_msg.linear_acceleration.x = imu_data['accX']
				self.imu_msg.linear_acceleration.y = imu_data['accY']
				self.imu_msg.linear_acceleration.z = imu_data['accZ']
				self.imu_msg.linear_acceleration_covariance = (0.0004, 0., 0., 0.,
						0.0004, 0., 0., 0., 0.0004)
			except KeyError:
				pass			
			try:
				self.pub_mag = True
				self.mag_msg.vector.x = imu_data['magX']
				self.mag_msg.vector.y = imu_data['magY']
				self.mag_msg.vector.z = imu_data['magZ']
			except KeyError:
				pass
		
		def fill_from_Vel(velocity_data):
			'''Fill messages with information from 'velocity' MTData block.'''
			self.pub_vel = True
			self.vel_msg.twist.linear.x = velocity_data['Vel_X']
			self.vel_msg.twist.linear.y = velocity_data['Vel_Y']
			self.vel_msg.twist.linear.z = velocity_data['Vel_Z']
		
		def fill_from_Orient(orient_data):
			'''Fill messages with information from 'orientation' MTData block.'''
			self.pub_imu = True
			if orient_data.has_key('quaternion'):
				w, x, y, z = orient_data['quaternion']
			elif orient_data.has_key('roll'):
				# FIXME check that Euler angles are in radians
				x, y, z, w = quaternion_from_euler(radians(orient_data['roll']),
						radians(orient_data['pitch']), radians(orient_data['yaw']))
			elif orient_data.has_key('matrix'):
				m = identity_matrix()
				m[:3,:3] = orient_data['matrix']
				x, y, z, w = quaternion_from_matrix(m)
			self.imu_msg.orientation.x = x
			self.imu_msg.orientation.y = y
			self.imu_msg.orientation.z = z
			self.imu_msg.orientation.w = w
			self.imu_msg.orientation_covariance = (radians(1.), 0., 0., 0.,
					radians(1.), 0., 0., 0., radians(9.))
		
		def fill_from_Pos(position_data):
			'''Fill messages with information from 'position' MTData block.'''
			self.pub_gps = True
			self.xgps_msg.latitude = self.gps_msg.latitude = position_data['Lat']
			self.xgps_msg.longitude = self.gps_msg.longitude = position_data['Lon']
			self.xgps_msg.altitude = self.gps_msg.altitude = position_data['Alt']
		
		def fill_from_Stat(status):
			'''Fill messages with information from 'status' MTData block.'''
			self.pub_diag = True
			if status & 0b0001:
				self.stest_stat.level = DiagnosticStatus.OK
				self.stest_stat.message = "Ok"
			else:
				self.stest_stat.level = DiagnosticStatus.ERROR
				self.stest_stat.message = "Failed"
			if status & 0b0010:
				self.xkf_stat.level = DiagnosticStatus.OK
				self.xkf_stat.message = "Valid"
			else:
				self.xkf_stat.level = DiagnosticStatus.WARN
				self.xkf_stat.message = "Invalid"
			if status & 0b0100:
				self.gps_stat.level = DiagnosticStatus.OK
				self.gps_stat.message = "Ok"
				self.gps_msg.status.status = NavSatStatus.STATUS_FIX
				self.xgps_msg.status.status = GPSStatus.STATUS_FIX
				self.gps_msg.status.service = NavSatStatus.SERVICE_GPS
				self.xgps_msg.status.position_source = 0b01101001
				self.xgps_msg.status.motion_source = 0b01101010
				self.xgps_msg.status.orientation_source = 0b01101010
			else:
				self.gps_stat.level = DiagnosticStatus.WARN
				self.gps_stat.message = "No fix"
				self.gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
				self.xgps_msg.status.status = GPSStatus.STATUS_NO_FIX
				self.gps_msg.status.service = 0
				self.xgps_msg.status.position_source = 0b01101000
				self.xgps_msg.status.motion_source = 0b01101000
				self.xgps_msg.status.orientation_source = 0b01101000

		def fill_from_Auxiliary(aux_data):
			'''Fill messages with information from 'Auxiliary' MTData block.'''
			try:
				self.anin1_msg.data = o['Ain_1']
				self.pub_anin1 = True
			except KeyError:
				pass
			try:
				self.anin2_msg.data = o['Ain_2']
				self.pub_anin2 = True
			except KeyError:
				pass

		def fill_from_Temperature(o):
			'''Fill messages with information from 'Temperature' MTData2 block.'''
			self.pub_temp = True
			self.temp_msg.data = o['Temp']
		
		def fill_from_Timestamp(o):
			'''Fill messages with information from 'Timestamp' MTData2 block.'''
			try:
				# put timestamp from gps UTC time if available
				y, m, d, hr, mi, s, ns, f = o['Year'], o['Month'], o['Day'],\
						o['Hour'], o['Minute'], o['Second'], o['ns'], o['Flags']
				if f&0x4:
					secs = time.mktime((y, m, d, hr, mi, s, 0, 0, 0))
					self.h.stamp.secs = secs
					self.h.stamp.nsecs = ns
			except KeyError:
				pass
			# TODO find what to do with other kind of information
			pass
		
		def fill_from_Orientation_Data(o):
			'''Fill messages with information from 'Orientation Data' MTData2 block.'''
			self.pub_imu = True
			try:
				x, y, z, w = o['Q1'], o['Q2'], o['Q3'], o['Q0']
			except KeyError:
				pass
			try: 
				# FIXME check that Euler angles are in radians
				x, y, z, w = quaternion_from_euler(radians(o['Roll']),
						radians(o['Pitch']), radians(o['Yaw']))
			except KeyError:
				pass
			try:
				a, b, c, d, e, f, g, h, i = o['a'], o['b'], o['c'], o['d'],\
						o['e'], o['f'], o['g'], o['h'], o['i']
				m = identity_matrix()
				m[:3,:3] = ((a, b, c), (d, e, f), (g, h, i))
				x, y, z, w = quaternion_from_matrix(m)
			except KeyError:
				pass
			self.imu_msg.orientation.x = x
			self.imu_msg.orientation.y = y
			self.imu_msg.orientation.z = z
			self.imu_msg.orientation.w = w
			self.imu_msg.orientation_covariance = (radians(1.), 0., 0., 0.,
					radians(1.), 0., 0., 0., radians(9.))
		
		def fill_from_Pressure(o):
			'''Fill messages with information from 'Pressure' MTData2 block.'''
			self.press_msg.data = o['Pressure']
		
		def fill_from_Acceleration(o):
			'''Fill messages with information from 'Acceleration' MTData2 block.'''
			self.pub_imu = True
			
			# FIXME not sure we should treat all in that same way
			try:
				x, y, z = o['Delta v.x'], o['Delta v.y'], o['Delta v.z']
			except KeyError:
				pass
			try:
				x, y, z = o['freeAccX'], o['freeAccY'], o['freeAccZ']
			except KeyError:
				pass
			try:
				x, y, z = o['accX'], o['accY'], o['accZ']
			except KeyError:
				pass
			      
			self.imu_msg.linear_acceleration.x = x
			self.imu_msg.linear_acceleration.y = y
			self.imu_msg.linear_acceleration.z = z
			self.imu_msg.linear_acceleration_covariance = (0.0004, 0., 0., 0.,
					0.0004, 0., 0., 0., 0.0004)
		
		def fill_from_Position(o):
			'''Fill messages with information from 'Position' MTData2 block.'''
			# TODO publish ECEF
			try:
				self.xgps_msg.latitude = self.gps_msg.latitude = o['lat']
				self.xgps_msg.longitude = self.gps_msg.longitude = o['lon']
				self.pub_gps = True
				alt = o.get('altMsl', o.get('altEllipsoid', 0))
				self.xgps_msg.altitude = self.gps_msg.altitude = alt
			except KeyError:
				pass
		
		def fill_from_Angular_Velocity(o):
			'''Fill messages with information from 'Angular Velocity' MTData2 block.'''
			try:
				self.imu_msg.angular_velocity.x = o['gyrX']
				self.imu_msg.angular_velocity.y = o['gyrY']
				self.imu_msg.angular_velocity.z = o['gyrZ']
				self.imu_msg.angular_velocity_covariance = (radians(0.025), 0., 0., 0.,
						radians(0.025), 0., 0., 0., radians(0.025))
				self.pub_imu = True
				self.vel_msg.twist.angular.x = o['gyrX']
				self.vel_msg.twist.angular.y = o['gyrY']
				self.vel_msg.twist.angular.z = o['gyrZ']
				self.pub_vel = True
			except KeyError:
				pass
			# TODO decide what to do with 'Delta q'

		def fill_from_GPS(o):
			'''Fill messages with information from 'GPS' MTData2 block.'''
			try:	# DOP
				self.xgps_msg.gdop = o['gDOP']
				self.xgps_msg.pdop = o['pDOP']
				self.xgps_msg.hdop = o['hDOP']
				self.xgps_msg.vdop = o['vDOP']
				self.xgps_msg.tdop = o['tDOP']
				self.pub_gps = True
			except KeyError:
				pass
			try:	# Time UTC
				y, m, d, hr, mi, s, ns, f = o['year'], o['month'], o['day'],\
						o['hour'], o['min'], o['sec'], o['nano'], o['valid']
				if f&0x4:
					secs = time.mktime((y, m, d, hr, mi, s, 0, 0, 0))
					self.h.stamp.secs = secs
					self.h.stamp.nsecs = ns
			except KeyError:
				pass
			# TODO publish SV Info

		def fill_from_SCR(o):
			'''Fill messages with information from 'SCR' MTData2 block.'''
			# TODO that's raw information
			pass

		def fill_from_Analog_In(o):
			'''Fill messages with information from 'Analog In' MTData2 block.'''
			try:
				self.anin1_msg.data = o['analogIn1']
				self.pub_anin1 = True
			except KeyError:
				pass
			try:
				self.anin2_msg.data = o['analogIn2']
				self.pub_anin2 = True
			except KeyError:
				pass

		def fill_from_Magnetic(o):
			'''Fill messages with information from 'Magnetic' MTData2 block.'''
			self.mag_msg.vector.x = o['magX']
			self.mag_msg.vector.y = o['magY']
			self.mag_msg.vector.z = o['magZ']
			self.pub_mag = True

		def fill_from_Velocity(o):
			'''Fill messages with information from 'Velocity' MTData2 block.'''
			self.vel_msg.twist.linear.x = o['velX']
			self.vel_msg.twist.linear.y = o['velY']
			self.vel_msg.twist.linear.z = o['velZ']
			self.pub_vel = True

		def fill_from_Status(o):
			'''Fill messages with information from 'Status' MTData2 block.'''
			try:
				status = o['StatusByte']
				fill_from_Stat(status)
			except KeyError:
				pass
			try:
				status = o['StatusWord']
				fill_from_Stat(status)
			except KeyError:
				pass
			# TODO RSSI

		def fill_from_Sample(o):
			'''Catch 'Sample' MTData blocks.'''
			rospy.logdebug("Got MTi data packet: 'Sample', ignored!")

		def find_handler_name(name):
			return "fill_from_%s"%(name.replace(" ", "_"))

		# get data
		data = self.mt.read_measurement()
		
		# fill messages based on available data fields
		for n, o in data.items():
			try:
				locals()[find_handler_name(n)](o)
			except KeyError:
				rospy.logwarn("Unknown MTi data packet: '%s', ignoring."%n)

		# publish available information
		if self.pub_imu:
			self.imu_msg.header = self.h
			self.imu_pub.publish(self.imu_msg)
		if self.pub_gps:
			self.xgps_msg.header = self.gps_msg.header = self.h
			self.gps_pub.publish(self.gps_msg)
			self.xgps_pub.publish(self.xgps_msg)
		if self.pub_vel:
			self.vel_msg.header = self.h
			self.vel_pub.publish(self.vel_msg)
		if self.pub_mag:
			self.mag_msg.header = self.h
			self.mag_pub.publish(self.mag_msg)
		if self.pub_temp:
			self.temp_pub.publish(self.temp_msg)
		if self.pub_press:
			self.press_pub.publish(self.press_msg)
		if self.pub_anin1:
			self.analog_in1_pub.publish(self.anin1_msg)
		if self.pub_anin2:
			self.analog_in2_pub.publish(self.anin2_msg)
		if self.pub_diag:
			self.diag_msg.header = self.h
			self.diag_pub.publish(self.diag_msg)
		# publish string representation
		self.str_pub.publish(str(data))



def main():
	'''Create a ROS node and instantiate the class.'''
	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	driver.spin()


if __name__== '__main__':
	main()
	
		
