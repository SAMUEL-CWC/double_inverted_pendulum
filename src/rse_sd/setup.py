from setuptools import find_packages, setup

package_name = 'rse_sd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='samuelchien2000@gmail.com',
    description='Self driving package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_node = rse_sd.command_node:main',
            'dashcam_node = rse_sd.dashcam_node:main',
            'class_counts_client = rse_sd.class_counts_client:main',
            'class_counts_server = rse_sd.class_counts_server:main',
            'find_object_client = rse_sd.find_object_client:main',
            'find_object_server = rse_sd.find_object_server:main',
        ],
    },
)
