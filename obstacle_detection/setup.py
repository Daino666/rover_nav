import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'obstacle_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsm',
    maintainer_email='vighneshdeshmukh66@gmail.com',
    description='Obstacle detection pipeline for LeapOne rover (D435i)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_filter = obstacle_detection.obstacle_filter:main',
            'obstacle_clustering = obstacle_detection.obstacle_clustering:main',
        ],
    },
)
