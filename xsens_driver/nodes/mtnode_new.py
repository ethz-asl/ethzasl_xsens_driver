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
		
		self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
		self.diag_msg = DiagnosticArray()
		self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1,
				message='No status information')
		self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1,
				message='No status information')
		self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1,
				message='No status information')
		self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]

		self.imu_pub = rospy.Publisher('imu/data', Imu)
		self.gps_pub = rospy.Publisher('fix', NavSatFix)
		self.xgps_pub = rospy.Publisher('fix_extended', GPSFix)
		self.vel_pub = rospy.Publisher('velocity', TwistStamped)
		self.mag_pub = rospy.Publisher('magnetic', Vector3Stamped)
		self.temp_pub = rospy.Publisher('temperature', Float32)	# decide type+header
		self.press_pub = rospy.Publisher('pressure', Float32) # decide type+header
		self.analog_in1_pub = rospy.Publisher('analog_in1', UInt16) # decide type+header
		self.analog_in2_pub = rospy.Publisher('analog_in2', UInt16) # decide type+header
		# TODO pressure, ITOW from raw GPS?
		self.old_bGPS = 256	# publish GPS only if new

		# publish a string version of all data; to be parsed by clients
		self.str_pub = rospy.Publisher('imu_data_str', String)



	def spin(self):
		try:
			while not rospy.is_shutdown():
				self.spin_once()
		# Ctrl-C signal interferes with select with the ROS signal handler
		# should be OSError in python 3.?
		except select.error:
			pass

	def spin_once(self):
		'''Read data from device and publishes ROS messages.'''
		# common header
		h = Header()
		h.stamp = rospy.Time.now()
		h.frame_id = self.frame_id
		
		# create messages and default values
		imu_msg = Imu()
		imu_msg.orientation_covariance = (-1., )*9
		imu_msg.angular_velocity_covariance = (-1., )*9
		imu_msg.linear_acceleration_covariance = (-1., )*9
		pub_imu = False
		gps_msg = NavSatFix()
		xgps_msg = GPSFix()
		pub_gps = False
		vel_msg = TwistStamped()
		pub_vel = False
		mag_msg = Vector3Stamped()
		pub_mag = False
		temp_msg = Float32()
		pub_temp = False
		press_msg = Float32()
		pub_press = False
		anin1_msg = UInt16()
		pub_anin1 = False
		anin2_msg = UInt16()
		pub_anin2 = False
		pub_diag = False
		
		def fill_from_raw(raw_data):
			'''Fill messages with information from 'raw' MTData block.'''
			# don't publish raw imu data anymore
			# TODO find what to do with that
			pass
		
		def fill_from_rawgps(rawgps_data):
			'''Fill messages with information from 'rawgps' MTData block.'''
			global pub_hps, xgps_msg, gps_msg
			if rawgps_data['bGPS']<self.old_bGPS:
				pub_gps = True
				# LLA
				xgps_msg.latitude = gps_msg.latitude = rawgps_data['LAT']*1e-7
				xgps_msg.longitude = gps_msg.longitude = rawgps_data['LON']*1e-7
				xgps_msg.altitude = gps_msg.altitude = rawgps_data['ALT']*1e-3
				# NED vel # TODO?
				# Accuracy
				# 2 is there to go from std_dev to 95% interval
				xgps_msg.err_horz = 2*rawgps_data['Hacc']*1e-3
				xgps_msg.err_vert = 2*rawgps_data['Vacc']*1e-3
			self.old_bGPS = rawgps_data['bGPS']
		
		def fill_from_Temp(temp):
			'''Fill messages with information from 'temperature' MTData block.'''
			global pub_temp, temp_msg
			pub_temp = True
			temp_msg.data = temp
		
		def fill_from_Calib(imu_data):
			'''Fill messages with information from 'calibrated' MTData block.'''
			global pub_imu, imu_msg, pub_vel, vel_msg, pub_mag, mag_msg
			try:
				pub_imu = True
				imu_msg.angular_velocity.x = imu_data['gyrX']
				imu_msg.angular_velocity.y = imu_data['gyrY']
				imu_msg.angular_velocity.z = imu_data['gyrZ']
				imu_msg.angular_velocity_covariance = (radians(0.025), 0., 0., 0.,
						radians(0.025), 0., 0., 0., radians(0.025))
				pub_vel = True
				vel_msg.twist.angular.x = imu_data['gyrX']
				vel_msg.twist.angular.y = imu_data['gyrY']
				vel_msg.twist.angular.z = imu_data['gyrZ']
			except KeyError:
				pass
			try:
				pub_imu = True
				imu_msg.linear_acceleration.x = imu_data['accX']
				imu_msg.linear_acceleration.y = imu_data['accY']
				imu_msg.linear_acceleration.z = imu_data['accZ']
				imu_msg.linear_acceleration_covariance = (0.0004, 0., 0., 0.,
						0.0004, 0., 0., 0., 0.0004)
			except KeyError:
				pass			
			try:
				pub_mag = True
				mag_msg.vector.x = imu_data['magX']
				mag_msg.vector.y = imu_data['magY']
				mag_msg.vector.z = imu_data['magZ']
			except KeyError:
				pass
		
		def fill_from_Vel(velocity_data):
			'''Fill messages with information from 'velocity' MTData block.'''
			global pub_vel, vel_msg
			pub_vel = True
			vel_msg.twist.linear.x = velocity_data['Vel_X']
			vel_msg.twist.linear.y = velocity_data['Vel_Y']
			vel_msg.twist.linear.z = velocity_data['Vel_Z']
		
		def fill_from_Orient(orient_data):
			'''Fill messages with information from 'orientation' MTData block.'''
			global pub_imu, imu_msg
			pub_imu = True
			if orient.has_key('quaternion'):
				w, x, y, z = orient['quaternion']
			elif orient.has_key('roll'):
				# FIXME check that Euler angles are in radians
				x, y, z, w = quaternion_from_euler(radians(orient['roll']),
						radians(orient['pitch']), radians(orient['yaw']))
			elif orient.has_key('matrix'):
				m = identity_matrix()
				m[:3,:3] = orient['matrix']
				x, y, z, w = quaternion_from_matrix(m)
			imu_msg.orientation.x = x
			imu_msg.orientation.y = y
			imu_msg.orientation.z = z
			imu_msg.orientation.w = w
			imu_msg.orientation_covariance = (radians(1.), 0., 0., 0.,
					radians(1.), 0., 0., 0., radians(9.))
		
		def fill_from_Pos(position_data):
			'''Fill messages with information from 'position' MTData block.'''
			global pub_gps, xgps_msg, gps_msg
			pub_gps = True
			xgps_msg.latitude = gps_msg.latitude = position_data['Lat']
			xgps_msg.longitude = gps_msg.longitude = position_data['Lon']
			xgps_msg.altitude = gps_msg.altitude = position_data['Alt']
		
		def fill_from_Stat(status):
			'''Fill messages with information from 'status' MTData block.'''
			global pub_diag, pub_gps, gps_msg, xgps_msg
			pub_diag = True
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
				gps_msg.status.status = NavSatStatus.STATUS_FIX
				xgps_msg.status.status = GPSStatus.STATUS_FIX
				gps_msg.status.service = NavSatStatus.SERVICE_GPS
				xgps_msg.status.position_source = 0b01101001
				xgps_msg.status.motion_source = 0b01101010
				xgps_msg.status.orientation_source = 0b01101010
			else:
				self.gps_stat.level = DiagnosticStatus.WARN
				self.gps_stat.message = "No fix"
				gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
				xgps_msg.status.status = GPSStatus.STATUS_NO_FIX
				gps_msg.status.service = 0
				xgps_msg.status.position_source = 0b01101000
				xgps_msg.status.motion_source = 0b01101000
				xgps_msg.status.orientation_source = 0b01101000

		def fill_from_Auxiliary(aux_data):
			'''Fill messages with information from 'Auxiliary' MTData block.'''
			global pub_anin1, pub_anin2, anin1_msg, anin2_msg
			try:
				anin1_msg.data = o['Ain_1']
				pub_anin1 = True
			except KeyError:
				pass
			try:
				anin2_msg.data = o['Ain_2']
				pub_anin2 = True
			except KeyError:
				pass

		def fill_from_Temperature(o):
			'''Fill messages with information from 'Temperature' MTData2 block.'''
			global pub_temp, temp_msg
			pub_temp = True
			temp_msg.data = o['Temp']
		
		def fill_from_Timestamp(o):
			'''Fill messages with information from 'Timestamp' MTData2 block.'''
			global h
			try:
				# put timestamp from gps UTC time if available
				y, m, d, hr, mi, s, ns, f = o['Year'], o['Month'], o['Day'],\
						o['Hour'], o['Minute'], o['Second'], o['ns'], o['Flags']
				if f&0x4:
					secs = time.mktime((y, m, d, hr, mi, s, 0, 0, 0))
					h.stamp.secs = secs
					h.stamp.nsecs = ns
			except KeyError:
				pass
			# TODO find what to do with other kind of information
			pass
		
		def fill_from_Orientation_Data(o):
			'''Fill messages with information from 'Orientation Data' MTData2 block.'''
			global pub_imu, imu_msg
			pub_imu = True
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
			imu_msg.orientation.x = x
			imu_msg.orientation.y = y
			imu_msg.orientation.z = z
			imu_msg.orientation.w = w
			imu_msg.orientation_covariance = (radians(1.), 0., 0., 0.,
					radians(1.), 0., 0., 0., radians(9.))
		
		def fill_from_Pressure(o):
			'''Fill messages with information from 'Pressure' MTData2 block.'''
			global pub_press, press_msg
			press_msg.data = o['Pressure']
		
		def fill_from_Acceleration(o):
			'''Fill messages with information from 'Acceleration' MTData2 block.'''
			global pub_imu, imu_msg
			pub_imu = True
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
			imu_msg.linear_acceleration.x = x
			imu_msg.linear_acceleration.y = y
			imu_msg.linear_acceleration.z = z
			imu_msg.linear_acceleration_covariance = (0.0004, 0., 0., 0.,
					0.0004, 0., 0., 0., 0.0004)
		
		def fill_from_Position(o):
			'''Fill messages with information from 'Position' MTData2 block.'''
			global pub_gps, xgps_msg, gps_msg
			# TODO publish ECEF
			try:
				xgps_msg.latitude = gps_msg.latitude = o['lat']
				xgps_msg.longitude = gps_msg.longitude = o['lon']
				pub_gps = True
				alt = o.get('altMsl', o.get('altEllipsoid', 0))
				xgps_msg.altitude = gps_msg.altitude = alt
			except KeyError:
				pass
		
		def fill_from_Angular_Velocity(o):
			'''Fill messages with information from 'Angular Velocity' MTData2 block.'''
			global pub_imu, imu_msg, pub_vel, vel_msg
			try:
				imu_msg.angular_velocity.x = o['gyrX']
				imu_msg.angular_velocity.y = o['gyrY']
				imu_msg.angular_velocity.z = o['gyrZ']
				imu_msg.angular_velocity_covariance = (radians(0.025), 0., 0., 0.,
						radians(0.025), 0., 0., 0., radians(0.025))
				pub_imu = True
				vel_msg.twist.angular.x = o['gyrX']
				vel_msg.twist.angular.y = o['gyrY']
				vel_msg.twist.angular.z = o['gyrZ']
				pub_vel = True
			except KeyError:
				pass
			# TODO decide what to do with 'Delta q'

		def fill_from_GPS(o):
			'''Fill messages with information from 'GPS' MTData2 block.'''
			global pub_gps, h, xgps_msg
			try:	# DOP
				xgps_msg.gdop = o['gDOP']
				xgps_msg.pdop = o['pDOP']
				xgps_msg.hdop = o['hDOP']
				xgps_msg.vdop = o['vDOP']
				xgps_msg.tdop = o['tDOP']
				pub_gps = True
			except KeyError:
				pass
			try:	# Time UTC
				y, m, d, hr, mi, s, ns, f = o['year'], o['month'], o['day'],\
						o['hour'], o['min'], o['sec'], o['nano'], o['valid']
				if f&0x4:
					secs = time.mktime((y, m, d, hr, mi, s, 0, 0, 0))
					h.stamp.secs = secs
					h.stamp.nsecs = ns
			except KeyError:
				pass
			# TODO publish SV Info

		def fill_from_SCR(o):
			'''Fill messages with information from 'SCR' MTData2 block.'''
			# TODO that's raw information
			pass

		def fill_from_Analog_In(o):
			'''Fill messages with information from 'Analog In' MTData2 block.'''
			global pub_anin1, pub_anin2, anin1_msg, anin2_msg
			try:
				anin1_msg.data = o['analogIn1']
				pub_anin1 = True
			except KeyError:
				pass
			try:
				anin2_msg.data = o['analogIn2']
				pub_anin2 = True
			except KeyError:
				pass

		def fill_from_Magnetic(o):
			'''Fill messages with information from 'Magnetic' MTData2 block.'''
			global pub_mag, mag_msg
			mag_msg.vector.x = o['magX']
			mag_msg.vector.y = o['magY']
			mag_msg.vector.z = o['magZ']
			pub_mag = True

		def fill_from_Velocity(o):
			'''Fill messages with information from 'Velocity' MTData2 block.'''
			global pub_vel, vel_msg
			vel_msg.twist.linear.x = o['velX']
			vel_msg.twist.linear.y = o['velY']
			vel_msg.twist.linear.z = o['velZ']
			pub_vel = True

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

		def find_handler_name(name):
			return "fill_from_%s"%(name.replace(" ", "_"))

		# get data
		data = self.mt.read_measurement()
		# fill messages based on available data fields
		for n, o in data:
			try:
				locals()[find_handler_name(n)](o)
			except KeyError:
				rospy.logwarn("Unknown MTi data packet: '%s', ignoring."%n)

		# publish available information
		if pub_imu:
			imu_msg.header = h
			self.imu_pub.publish(imu_msg)
		if pub_gps:
			xgps_msg.header = gps_msg.header = h
			self.gps_pub.publish(gps_msg)
			self.xgps_pub.publish(xgps_msg)
		if pub_vel:
			vel_msg.header = h
			self.vel_pub.publish(vel_msg)
		if pub_mag:
			mag_msg.header = h
			self.mag_pub.publish(mag_msg)
		if pub_temp:
			self.temp_pub.publish(temp_msg)
		if pub_press:
			self.press_pub.publish(press_msg)
		if pub_anin1:
			self.analog_in1_pub.publish(anin1_msg)
		if pub_anin2:
			self.analog_in2_pub.publish(anin2_msg)
		if pub_diag:
			self.diag_msg.header = h
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
	
		
