from setuptools import setup
import os
from glob import glob

package_name = 'jax_hardware'

setup(
    name=package_name,
    version='0.0.1',
    # This picks up both the outer folder and any python modules inside
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Includes all launch files for bringup
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tdp378',
    maintainer_email='tdp378@gmail.com',
    description='Hardware bridge and simulation sync for Jax',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The Real Robot Node
            'jax_hardware_bridge = jax_hardware.jax_hardware_node:main',
            # The RViz Sync Node
            'sim_bridge = jax_hardware.sim_bridge:main',
        ],
    },
)