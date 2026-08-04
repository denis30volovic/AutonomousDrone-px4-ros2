from setuptools import setup

package_name = 'px4_orbit'

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
    description='ROS2 node for circular orbit missions with automatic takeoff, orbit execution, and landing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'orbit_controller = px4_orbit.orbit_controller:main'
        ],
    },
) 