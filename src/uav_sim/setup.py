from setuptools import setup

package_name = 'uav_sim'

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
    description='UAV simulator (2D) with wind + battery',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_sim = uav_sim.sim_node:main',
        ],
    },
)
