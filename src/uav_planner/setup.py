from setuptools import setup

package_name = 'uav_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panagiotagrosd',
    maintainer_email='panagiotagrosdouli@gmail.com',
    description='UAV planner with risk-aware return-to-home',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_planner = uav_planner.planner_node:main',
        ],
    },
)
