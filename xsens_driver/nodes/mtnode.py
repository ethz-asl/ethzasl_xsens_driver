#!/usr/bin/env python
import roslib; roslib.load_manifest('xsens_driver')
import rospy
import select

import mtdevice

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from gps_common.msg import GPSFix, GPSStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


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
		self.temp_pub = rospy.Publisher('temperature', Float32)	# decide type
		# TODO pressure, ITOW from raw GPS?
		self.old_bGPS = 256	# publish GPS only if new



	def spin(self):
		try:
			while not rospy.is_shutdown():
				self.spin_once()
		# Ctrl-C signal interferes with select with the ROS signal handler
		# should be OSError in python 3.?
		except select.error:
			pass

	def spin_once(self):

		def quat_from_orient(orient):
			'''Build a quaternion from orientation data.'''
			try:
				w, x, y, z = orient['quaternion']
				return (x, y, z, w)
			except KeyError:
				pass
			try:
				return quaternion_from_euler(pi*orient['roll']/180.,
						pi*orient['pitch']/180, pi*orient['yaw']/180.)
			except KeyError:
				pass
			try:
				m = identity_matrix()
				m[:3,:3] = orient['matrix']
				return quaternion_from_matrix(m)
			except KeyError:
				pass

		# get data
		data = self.mt.read_measurement()
		# common header
		h = Header()
		h.stamp = rospy.Time.now()
		h.frame_id = self.frame_id
		
		# get data (None if not present)
		temp = data.get('Temp')	# float
		raw_data = data.get('RAW')
		imu_data = data.get('Calib')
		orient_data = data.get('Orient')
		velocity_data = data.get('Vel')
		position_data = data.get('Pos')
		rawgps_data = data.get('RAWGPS')
		status = data.get('Stat')	# int

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
		
		# fill information where it's due
		# start by raw information that can be overriden
		if raw_data: # TODO warn about data not calibrated
			pub_imu = True
			pub_vel = True
			pub_mag = True
			pub_temp = True
			# acceleration
			imu_msg.linear_acceleration.x = raw_data['accX']
			imu_msg.linear_acceleration.y = raw_data['accY']
			imu_msg.linear_acceleration.z = raw_data['accZ']
			imu_msg.linear_acceleration_covariance = (0., )*9
			# gyroscopes
			imu_msg.angular_velocity.x = raw_data['gyrX']
			imu_msg.angular_velocity.y = raw_data['gyrY']
			imu_msg.angular_velocity.z = raw_data['gyrZ']
			imu_msg.angular_velocity_covariance = (0., )*9
			vel_msg.twist.angular.x = raw_data['gyrX']
			vel_msg.twist.angular.y = raw_data['gyrY']
			vel_msg.twist.angular.z = raw_data['gyrZ']
			# magnetometer
			mag_msg.vector.x = raw_data['magX']
			mag_msg.vector.y = raw_data['magY']
			mag_msg.vector.z = raw_data['magZ']
			# temperature
			# 2-complement decoding and 1/256 resolution
			x = raw_data['temp']
			if x&0x8000:
				temp_msg.data = (x - 1<<16)/256.
			else:
				temp_msg.data = x/256.
		if rawgps_data:
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
		if temp is not None:
			pub_temp = True
			temp_msg.data = temp
		if imu_data:
			try:
				imu_msg.angular_velocity.x = imu_data['gyrX']
				imu_msg.angular_velocity.y = imu_data['gyrY']
				imu_msg.angular_velocity.z = imu_data['gyrZ']
				imu_msg.angular_velocity_covariance = (radians(0.025), 0., 0., 0.,
						radians(0.025), 0., 0., 0., radians(0.025))
				pub_imu = True
				vel_msg.twist.angular.x = imu_data['gyrX']
				vel_msg.twist.angular.y = imu_data['gyrY']
				vel_msg.twist.angular.z = imu_data['gyrZ']
				pub_vel = True
			except KeyError:
				pass
			try:
				imu_msg.linear_acceleration.x = imu_data['accX']
				imu_msg.linear_acceleration.y = imu_data['accY']
				imu_msg.linear_acceleration.z = imu_data['accZ']
				imu_msg.linear_acceleration_covariance = (0.0004, 0., 0., 0.,
						0.0004, 0., 0., 0., 0.0004)
				pub_imu = True
			except KeyError:
				pass			
			try:
				mag_msg.vector.x = imu_data['magX']
				mag_msg.vector.y = imu_data['magY']
				mag_msg.vector.z = imu_data['magZ']
				pub_mag = True
			except KeyError:
				pass
		if velocity_data:
			pub_vel = True
			vel_msg.twist.linear.x = velocity_data['Vel_X']
			vel_msg.twist.linear.y = velocity_data['Vel_Y']
			vel_msg.twist.linear.z = velocity_data['Vel_Z']
		if orient_data:
			pub_imu = True
			orient_quat = quat_from_orient(orient_data)
			imu_msg.orientation.x = orient_quat[0]
			imu_msg.orientation.y = orient_quat[1]
			imu_msg.orientation.z = orient_quat[2]
			imu_msg.orientation.w = orient_quat[3]
			imu_msg.orientation_covariance = (radians(1.), 0., 0., 0.,
					radians(1.), 0., 0., 0., radians(9.))
		if position_data:
			pub_gps = True
			xgps_msg.latitude = gps_msg.latitude = position_data['Lat']
			xgps_msg.longitude = gps_msg.longitude = position_data['Lon']
			xgps_msg.altitude = gps_msg.altitude = position_data['Alt']
		if status is not None:
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
			else:
				self.gps_stat.level = DiagnosticStatus.WARN
				self.gps_stat.message = "No fix"
			self.diag_msg.header = h
			self.diag_pub.publish(self.diag_msg)

			if pub_gps:
				if status & 0b0100:
					gps_msg.status.status = NavSatStatus.STATUS_FIX
					xgps_msg.status.status = GPSStatus.STATUS_FIX
					gps_msg.status.service = NavSatStatus.SERVICE_GPS
					xgps_msg.status.position_source = 0b01101001
					xgps_msg.status.motion_source = 0b01101010
					xgps_msg.status.orientation_source = 0b01101010
				else:
					gps_msg.status.status = NavSatStatus.STATUS_NO_FIX
					xgps_msg.status.status = GPSStatus.STATUS_NO_FIX
					gps_msg.status.service = 0
					xgps_msg.status.position_source = 0b01101000
					xgps_msg.status.motion_source = 0b01101000
					xgps_msg.status.orientation_source = 0b01101000
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



def main():
	'''Create a ROS node and instantiate the class.'''
	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	driver.spin()


if __name__== '__main__':
	main()
	
		
