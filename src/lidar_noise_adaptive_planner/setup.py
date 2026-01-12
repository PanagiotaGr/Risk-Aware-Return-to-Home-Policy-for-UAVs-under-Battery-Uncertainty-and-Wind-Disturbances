from setuptools import setup
from glob import glob
import os

package_name = 'lidar_noise_adaptive_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panagiotagrosd',
    maintainer_email='panagiotagrosdouli@gmail.com',
    description='Adaptive LiDAR-based local planner parameter tuning',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'noise_adaptive_node = lidar_noise_adaptive_planner.noise_adaptive_node:main',
        ],
    },
)
