from setuptools import setup
import os
from glob import glob

package_name = 'clubbot2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install config directory
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shane',
    maintainer_email='your_email@example.com',
    description='ClubBot2 robot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = clubbot2.motor_controller:main',
            'hardware_interface = clubbot2.hardware_interface:main',
            # Add more nodes here later:
            # 'encoder_node = clubbot2.encoder_node:main',
            # 'bumper_node = clubbot2.bumper_node:main',
        ],
    },
)

