import os
from glob import glob
from setuptools import setup

package_name = 'ethzasl_xsens_driver_ros2'

setup(
    name='ethzasl_xsens_driver_ros2',
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Baril',
    maintainer_email='dominic.baril@norlab.ulaval.ca',
    description='ROS 2 Driver for XSens MT/MTi/MTi-G devices.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mti_node = ethzasl_xsens_driver_ros2.mtnode:main',
        ],
    },
)

