from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tb3_odometry'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aghyad Ali',
    maintainer_email='aghyad.ali.123@gmail.com',
    description='Open-loop square driving, wheel odometry and drift analysis for TurtleBot3.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_movement = tb3_odometry.square_movement:main',
            'wheel_odometry = tb3_odometry.wheel_odometry:main',
            'odom_plotter = tb3_odometry.odom_plotter:main',
        ],
    },
)
