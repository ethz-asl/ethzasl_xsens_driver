cmake_minimum_required(VERSION 3.10.2)
project(xsens_driver)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/mtnode.py
  nodes/mtdef.py
  nodes/mtdevice.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)