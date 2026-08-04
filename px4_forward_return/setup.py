from setuptools import setup

package_name = 'px4_forward_return'

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
    description='ROS2 node for complex forward-return missions with 360° rotation and automatic return to starting position',
    license='MIT',
    entry_points={
        'console_scripts': [
            'forward_return_controller = px4_forward_return.forward_return_controller:main'
        ],
    },
)