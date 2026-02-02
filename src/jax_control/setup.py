from setuptools import setup
import os
from glob import glob

package_name = 'jax_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tdp378',
    maintainer_email='todo@todo.com',
    description='Control logic for Jax the quadruped robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jax_controller = jax_control.Controller:main',
            'jax_teleop = jax_control.jax_teleop:main',
        ],
    },
)