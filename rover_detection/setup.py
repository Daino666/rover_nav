import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mukul-bimbra',
    maintainer_email='mukulbimbra321@gmail.com',
    description='ArUco landmark detection and rover global-position localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detect_roverpos = rover_detection.aruco_detect_roverpos:main',
        ],
    },
)
