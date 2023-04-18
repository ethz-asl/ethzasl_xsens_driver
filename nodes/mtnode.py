#!/usr/bin/env python3

import rospy
from xsens_driver._mtnode import XSensDriver

def main():
    '''Create a ROS node and instantiate the class.'''
    rospy.init_node('xsens_driver')
    driver = XSensDriver()
    driver.spin()

if __name__ == '__main__':
    main()