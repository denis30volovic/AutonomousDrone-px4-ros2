from setuptools import setup

package_name = 'px4_forward_flight'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Denis Volovik',
    maintainer_email='denis30volovic@gmail.com',
    description='ROS2 node for controlling drone forward and backward flight movements with smooth trajectory planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_flight_controller = px4_forward_flight.forward_flight_controller:main'
        ],
    },
)
