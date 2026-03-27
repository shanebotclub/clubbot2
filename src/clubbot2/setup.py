from setuptools import find_packages, setup

package_name = 'clubbot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/button_led_system.launch.py']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='shaned75@googlemail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 
             'mpu6050_node = clubbot2.mpu6050_node:main',
             'button_publisher = clubbot2.button_publisher:main',
             'led_subscriber = clubbot2.led_subscriber:main',
             'button_to_led_bridge = clubbot2.button_to_led_bridge:main',
             
        ],
    },
)
