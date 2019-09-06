^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package xsens_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.2 (2018-08-02)
------------------
* fix exception while closing node
* Contributors: Francis Colas

2.2.1 (2018-08-02)
------------------
* fix frame reorientation (only for orientation and linear velocity)
* fix skip-factor command line (`#80 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/80>`_)
* Contributors: Francis Colas

2.2.0 (2018-07-16)
------------------
* initial_wait argument for MTDevice, node and launch file
* separate gps_msg from Position Data
* handle both alignment rotation versions
* fix GetAlignmentRotation
* new timeout command line argument
* catch timeout exception while writing messages
* Feat/configurable covariance diagonals (`#65 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/65>`_)
* catch timeout exception during inspect (`#60 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/60>`_)
* Fix conversion from UTC to epoch time for /time_reference (`#56 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/56>`_)
* fix bug in sample time fine
* add missing package dependency on python-serial (`#54 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/54>`_)
* support delta q (`#49 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/49>`_)
* Synchronization settings and UTC time extension (`#46 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/46>`_)
* full list of error codes and messages
* Contributors: Alex Naiman, Atsushi Watanabe, Di Zeng, Francis Colas, Rein Appeldoorn, juichung kuo

2.1.0 (2017-04-14)
------------------
* Add no_rotation_duration option
* Fix typo (`#39 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/39>`_)
* Fix gnss pvt parsing (`#37 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/37>`_)
* fix GetOptionFlags (`#34 <https://github.com/ethz-asl/ethzasl_xsens_driver/issues/34>`_)
* Contributors: Andersw88, Francis Colas

2.0.1 (2016-08-16)
------------------
* fix TimeReference member name
* Contributors: Francis Colas

2.0.0 (2016-08-02)
------------------
* support of mark iv devices (configuration and ROS node)
* remove gps_common dependency (for jade and kinetic)
* work in 16.04 with pyserial3
* proper message types for temperature, pressure, magnetic field and time
* better timeout management
* various bug fixes
* Contributors: CTU robot, Francis Colas, João Sousa, Konstantinos Chatzilygeroudis, Latitude OBC, Vincent Rousseau, fcolas, jotaemesousa

1.0.3 (2014-05-14)
------------------
* Inclusion of launch file
* Additions and fixes from PAL robotics
* Add local frame conversion for calibrated imu data (acc, gyr, mag)
* Contributors: Enrique Fernandez, Francis Colas, Paul Mathieu, Sam Pfeiffer

1.0.2 (2014-03-04)
------------------
* catkinized
* experimental support of mark 4 IMUs
* fixed scaling in DOP values
* adding publisher for full data as a string
* relative topic names

1.0.1 (2012-08-27)
------------------
* minor improvements
* naming cleanup
* Contributors: Benjamin Hitov, Francis Colas, Nikolaus Demmel, Stéphane Magnenat, fcolas
