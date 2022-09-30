import os
from glob import glob
from setuptools import setup

package_name = 'xsens_driver'

setup(
    name='xsens_driver',
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francis Colas',
    maintainer_email='francis.colas@mavt.ethz.ch',
    description='ROS 2 driver for XSens MT/MTi/MTi-G devices.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'mtnode.py = xsens_driver.mtnode:main',
        ],
    },
)

