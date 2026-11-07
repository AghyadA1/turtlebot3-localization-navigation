from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'apf_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aghyad Ali',
    maintainer_email='aghyad.ali.123@gmail.com',
    description='AMCL + APF navigation for TurtleBot3 on ROS 2 Jazzy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'apf_controller = apf_nav.apf_controller:main',
        ],
    },
)
