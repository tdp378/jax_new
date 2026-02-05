from setuptools import setup
import os
from glob import glob

package_name = 'jax_hardware'

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
    maintainer='TDP378',
    description='Hardware interface',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jax_hardware_node = jax_hardware.jax_hardware_node:main'
        ],
    },
)