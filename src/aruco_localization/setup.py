from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ] + [
        (os.path.join('share', package_name, os.path.dirname(f)), [f])
        for f in glob('models/**/*', recursive=True) if os.path.isfile(f)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aghyad Ali',
    maintainer_email='aghyad.ali.123@gmail.com',
    description='Landmark-based global localization of a TurtleBot3 from ArUco markers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_localizer = aruco_localization.marker_localizer:main',
        ],
    },
)
