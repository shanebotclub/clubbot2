from setuptools import setup, find_packages

package_name = 'clubbot2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/button_led_system.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml', 'config/RobotParams.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shane',
    maintainer_email='your_email@example.com',
    description='Clubbot2 robot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'button_publisher = clubbot2.button_publisher:main',
            'button_to_led_bridge = clubbot2.button_to_led_bridge:main',
            'led_subscriber = clubbot2.led_subscriber:main',
            
        ],
    },
)
