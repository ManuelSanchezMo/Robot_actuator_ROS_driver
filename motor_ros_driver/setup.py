from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_ros_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), ['can_parser.dbc']),  # Specify the desired lib directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    setup_requires=['setuptools_scm'],
    include_package_data=True,
    maintainer='a',
    maintainer_email='manuel.sanchez.m@uma.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['motor_node = motor_ros_driver.motor_driver_node:main'],
         'console_scripts': ['talker_node = motor_ros_driver.talker:main'],
    },
)

