from setuptools import setup
import os
from glob import glob

package_name = 'dip_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # must match folder name
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel',
    maintainer_email='samuelbruin0618@g.ucla.edu',
    description='Simulation package for the double inverted pendulum',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulate = dip_sim.simulate:main',  # Entry point for the simulation node
        ],
    },
)
